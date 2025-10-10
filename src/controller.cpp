#include "controller_pkg/controller.hpp"
#include <tf2/utils.h>
#include <csignal>
#include <memory>
#include <thread>
#include <chrono>

namespace controller_pkg {

Controller::Controller() 
    : Node("controller"),
      path_received_(false),
      emergency_stop_(false),
      has_velocity_path_(false),
      last_target_index_(0),
      shutdown_requested_(false) {
    
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Controller initialized successfully");
}

Controller::~Controller() {
    // Ensure car stops when node is destroyed
    publish_stop_command();
}

bool Controller::initialize() {
    // Declare parameters
    this->declare_parameter("lookahead_distance", config_.lookahead_distance);
    this->declare_parameter("min_lookahead", config_.min_lookahead);
    this->declare_parameter("max_lookahead", config_.max_lookahead);
    this->declare_parameter("lookahead_ratio", config_.lookahead_ratio);
    this->declare_parameter("wheelbase", config_.wheelbase);
    this->declare_parameter("max_steering_angle", config_.max_steering_angle);
    this->declare_parameter("target_speed", config_.target_speed);
    this->declare_parameter("max_speed", config_.max_speed);
    this->declare_parameter("min_speed", config_.min_speed);
    this->declare_parameter("use_planner_velocity", config_.use_planner_velocity);
    this->declare_parameter("velocity_scale_factor", config_.velocity_scale_factor);
    this->declare_parameter("curvature_sensitivity", config_.curvature_sensitivity);
    this->declare_parameter("max_steering_rate", config_.max_steering_rate);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_link_frame", "base_link");

    // Get parameters
    config_.lookahead_distance = this->get_parameter("lookahead_distance").as_double();
    config_.min_lookahead = this->get_parameter("min_lookahead").as_double();
    config_.max_lookahead = this->get_parameter("max_lookahead").as_double();
    config_.lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    config_.wheelbase = this->get_parameter("wheelbase").as_double();
    config_.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    config_.target_speed = this->get_parameter("target_speed").as_double();
    config_.max_speed = this->get_parameter("max_speed").as_double();
    config_.min_speed = this->get_parameter("min_speed").as_double();
    config_.use_planner_velocity = this->get_parameter("use_planner_velocity").as_bool();
    config_.velocity_scale_factor = this->get_parameter("velocity_scale_factor").as_double();
    config_.curvature_sensitivity = this->get_parameter("curvature_sensitivity").as_double();
    config_.max_steering_rate = this->get_parameter("max_steering_rate").as_double();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Using TF: %s -> %s", map_frame_.c_str(), base_link_frame_.c_str());

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10);

    
    // Initialize subscribers (only waypoint_array and odom)
    waypoint_array_sub_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/planned_waypoints", 10,
        std::bind(&Controller::waypoint_array_callback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&Controller::odom_callback, this, std::placeholders::_1));
    
    // Initialize control timer (50 Hz)
    auto timer_period = std::chrono::milliseconds(20);
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&Controller::control_timer_callback, this));
    
    last_path_time_ = this->get_clock()->now();
    last_control_time_ = this->get_clock()->now();
    last_steering_angle_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Controller subscribed to /planned_waypoints topic");
    
    return true;
}

void Controller::waypoint_array_callback(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg) {
    if (msg->wpnts.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty waypoint array");
        return;
    }
    
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    // Store the waypoint array
    current_waypoints_ = *msg;
    has_velocity_path_ = true;
    
    path_received_ = true;
    last_path_time_ = this->get_clock()->now();
    last_target_index_ = 0;  // Reset target index for new path
    
    RCLCPP_INFO(this->get_logger(), "Received waypoint array with %zu points",
                 current_waypoints_.wpnts.size());
}

void Controller::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Get velocity from odom message
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    vehicle_state_.velocity = std::sqrt(vx*vx + vy*vy);

    // Get position and orientation from TF (map_frame_ -> base_link_frame_)
    try {
        auto transform = tf_buffer_->lookupTransform(
            map_frame_, base_link_frame_,
            tf2::TimePointZero  // Get latest available transform
        );

        // Extract position
        vehicle_state_.x = transform.transform.translation.x;
        vehicle_state_.y = transform.transform.translation.y;

        // Extract orientation (yaw)
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        vehicle_state_.yaw = yaw;

        vehicle_state_.valid = true;

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Could not get %s->%s transform: %s", map_frame_.c_str(), base_link_frame_.c_str(), ex.what());
        vehicle_state_.valid = false;
    }
}

void Controller::control_timer_callback() {
    // Check if shutdown was requested
    if (shutdown_requested_.load()) {
        publish_stop_command();
        return;
    }
    
    if (!vehicle_state_.valid || !path_received_) {
        // Send stop command if no valid state or path
        publish_stop_command();
        return;
    }
    
    // Check if path is too old (emergency stop after 2 seconds)
    auto current_time = this->get_clock()->now();
    auto path_age = (current_time - last_path_time_).seconds();
    if (path_age > 2.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 
                             1000, "Path is too old, stopping vehicle");
        emergency_stop_ = true;
    } else {
        emergency_stop_ = false;
    }
    
    if (emergency_stop_) {
        publish_stop_command();
        return;
    }
    
    // Use Pure Pursuit controller
    auto [steering_angle, speed] = pure_pursuit_control();
    
    // Apply steering rate limiting
    double dt = (current_time - last_control_time_).seconds();
    if (dt > 0.0 && dt < 0.1) {  // Valid time step
        double max_change = config_.max_steering_rate * dt;
        double steering_change = steering_angle - last_steering_angle_;
        if (std::abs(steering_change) > max_change) {
            steering_angle = last_steering_angle_ + std::copysign(max_change, steering_change);
        }
    }
    
    // Update control history
    last_steering_angle_ = steering_angle;
    last_control_time_ = current_time;

    // Create and publish drive message
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = current_time;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    
    drive_pub_->publish(drive_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Control: steering=%.3f, speed=%.3f", 
                 steering_angle, speed);
}

std::pair<double, double> Controller::pure_pursuit_control() {
    ae_hyu_msgs::msg::WpntArray waypoints;
    VehicleState vehicle;
    
    {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        waypoints = current_waypoints_;
        vehicle = vehicle_state_;
    }
    
    if (waypoints.wpnts.empty()) {
        return std::make_pair(0.0, 0.0);
    }
    
    int target_index = find_target_point(waypoints, vehicle);
    if (target_index < 0 || target_index >= static_cast<int>(waypoints.wpnts.size())) {
        RCLCPP_WARN(this->get_logger(), "No valid target point found");
        return std::make_pair(0.0, 0.0);
    }
    
    // Get target point coordinates from waypoint array
    double target_x = waypoints.wpnts[target_index].x_m;
    double target_y = waypoints.wpnts[target_index].y_m;
    double curvature = waypoints.wpnts[target_index].kappa_radpm;
    
    // Calculate steering angle
    double steering_angle = calculate_steering_angle(target_x, target_y, vehicle);
    
    // Get velocity from planner (already optimized for curvature)
    double speed = get_target_speed(target_index);
    
    RCLCPP_DEBUG(this->get_logger(), "Control: target=%d, steering=%.2f°, speed=%.1f, curvature=%.3f", 
                 target_index, steering_angle*180/M_PI, speed, curvature);
    
    // Clamp values
    steering_angle = std::clamp(steering_angle, -config_.max_steering_angle, 
                               config_.max_steering_angle);
    speed = std::clamp(speed, config_.min_speed, config_.max_speed);
    
    return std::make_pair(steering_angle, speed);
}


std::pair<int, double> Controller::find_vehicle_position_on_path(const ae_hyu_msgs::msg::WpntArray& waypoints,
                                                                  const VehicleState& vehicle) {
    if (waypoints.wpnts.empty()) {
        return std::make_pair(-1, 0.0);
    }
    
    // Find closest point first
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = 0;
    
    for (int i = 0; i < static_cast<int>(waypoints.wpnts.size()); ++i) {
        double dx = waypoints.wpnts[i].x_m - vehicle.x;
        double dy = waypoints.wpnts[i].y_m - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    // Use frenet coordinate directly from waypoint if available, otherwise calculate
    double vehicle_s = waypoints.wpnts[closest_index].s_m;
    
    if (closest_index < static_cast<int>(waypoints.wpnts.size()) - 1) {
        // Project vehicle position onto the line segment between closest and next point
        double x1 = waypoints.wpnts[closest_index].x_m;
        double y1 = waypoints.wpnts[closest_index].y_m;
        double x2 = waypoints.wpnts[closest_index + 1].x_m;
        double y2 = waypoints.wpnts[closest_index + 1].y_m;
        
        double path_dx = x2 - x1;
        double path_dy = y2 - y1;
        double segment_length_sq = path_dx*path_dx + path_dy*path_dy;
        
        if (segment_length_sq > 1e-6) {
            double vehicle_dx = vehicle.x - x1;
            double vehicle_dy = vehicle.y - y1;
            
            // Project onto segment (t = 0 at closest_index, t = 1 at next point)
            double t = (vehicle_dx * path_dx + vehicle_dy * path_dy) / segment_length_sq;
            t = std::clamp(t, 0.0, 1.0);
            
            // Interpolate s-coordinate based on projection
            double s1 = waypoints.wpnts[closest_index].s_m;
            double s2 = waypoints.wpnts[closest_index + 1].s_m;
            vehicle_s = s1 + t * (s2 - s1);
        }
    }
    
    return std::make_pair(closest_index, vehicle_s);
}

int Controller::find_target_point(const ae_hyu_msgs::msg::WpntArray& waypoints,
                                   const VehicleState& vehicle) {
    if (waypoints.wpnts.empty()) return -1;
    
    // Get vehicle's position along the path (Frenet s-coordinate)
    auto [closest_index, vehicle_s] = find_vehicle_position_on_path(waypoints, vehicle);
    if (closest_index < 0) return -1;
    
    // Use iterative approach to find target point considering curvature
    // Start with base lookahead and refine based on curvature
    double base_lookahead = vehicle.velocity * config_.lookahead_ratio;
    base_lookahead = std::clamp(base_lookahead, config_.min_lookahead, config_.max_lookahead);
    
    // Find initial target point with base lookahead
    double target_s = vehicle_s + base_lookahead;
    int initial_target = find_point_at_distance(waypoints, closest_index, target_s);
    
    // Get curvature at initial target point and adjust lookahead
    double curvature = get_curvature_at_index(initial_target);
    double adjusted_lookahead = calculate_lookahead_distance(vehicle.velocity, curvature);
    
    // Find final target point with adjusted lookahead
    target_s = vehicle_s + adjusted_lookahead;
    int final_target = find_point_at_distance(waypoints, closest_index, target_s);
    
    // Safety check: ensure we don't go beyond available curvature data
    if (final_target >= static_cast<int>(waypoints.wpnts.size())) {
        final_target = static_cast<int>(waypoints.wpnts.size()) - 1;
    }
    
    last_target_index_ = final_target;
    return final_target;
}

/////////////////////////////////////////////////////////////
double Controller::calculate_lookahead_distance(double velocity, double curvature) {
    double base_lookahead = velocity * config_.lookahead_ratio;
    // Reduce lookahead distance as curvature increases (using configurable sensitivity)
    double curvature_factor = std::max(1.0, std::abs(curvature) * config_.curvature_sensitivity);
    double lookahead = base_lookahead / curvature_factor;
    return std::clamp(lookahead, config_.min_lookahead, config_.max_lookahead);
}

double Controller::calculate_steering_angle(double target_x, double target_y, 
                                             const VehicleState& vehicle) {
    // Transform target point to vehicle coordinate frame
    double dx = target_x - vehicle.x;
    double dy = target_y - vehicle.y;
    
    // Rotate to vehicle frame
    double cos_yaw = std::cos(-vehicle.yaw);
    double sin_yaw = std::sin(-vehicle.yaw);
    double local_x = dx * cos_yaw - dy * sin_yaw;
    double local_y = dx * sin_yaw + dy * cos_yaw;
    
    // Calculate lookahead distance
    double lookahead_distance = std::sqrt(local_x*local_x + local_y*local_y);
    
    if (lookahead_distance < 0.1) {
        return 0.0;  // Too close, no steering
    }
    
    // Pure pursuit curvature calculation: 2y/d^2
    double curvature = 2.0 * local_y / (lookahead_distance * lookahead_distance);
    double steering_angle = std::atan(config_.wheelbase * curvature);
    
    return steering_angle;
}


double Controller::get_target_speed(int target_index) {
    // Use planner velocity if available
    double planner_velocity = get_velocity_at_point(target_index);
    
    if (config_.use_planner_velocity && planner_velocity > 0.0) {
        return planner_velocity * config_.velocity_scale_factor;
    }
    
    // Fallback to configured target speed
    return config_.target_speed;
}


double Controller::get_velocity_at_point(int target_index) const {
    if (!has_velocity_path_ || target_index < 0 ||
        target_index >= static_cast<int>(current_waypoints_.wpnts.size())) {
        return -1.0; // Invalid velocity
    }
    
    return current_waypoints_.wpnts[target_index].vx_mps;
}

void Controller::publish_stop_command() {
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->get_clock()->now();
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.speed = 0.0;
    drive_msg.drive.steering_angle = 0.0;
    drive_msg.drive.acceleration = -5.0;  // Emergency brake
    drive_msg.drive.jerk = 0.0;
    drive_msg.drive.steering_angle_velocity = 0.0;
    
    drive_pub_->publish(drive_msg);
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Publishing stop command - Vehicle stopped for safety");
}

void Controller::shutdown_handler() {
    RCLCPP_WARN(this->get_logger(), "Shutdown signal received - stopping vehicle safely");
    shutdown_requested_.store(true);
    
    // Publish multiple stop commands to ensure vehicle stops
    for (int i = 0; i < 5; ++i) {
        publish_stop_command();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

/////////////////////////////////////////////////////////////
// Helper function implementations
int Controller::find_point_at_distance(const ae_hyu_msgs::msg::WpntArray& waypoints, int start_index, double target_s) {
    if (waypoints.wpnts.empty() || start_index >= static_cast<int>(waypoints.wpnts.size())) {
        return start_index;
    }
    
    // Find waypoint with s-coordinate closest to target_s
    int target_index = start_index;
    double min_diff = std::abs(waypoints.wpnts[start_index].s_m - target_s);
    
    for (int i = start_index; i < static_cast<int>(waypoints.wpnts.size()); ++i) {
        double diff = std::abs(waypoints.wpnts[i].s_m - target_s);
        if (diff < min_diff) {
            min_diff = diff;
            target_index = i;
        }
        // If we've passed the target s, stop searching
        if (waypoints.wpnts[i].s_m > target_s) {
            break;
        }
    }
    
    return std::min(target_index, static_cast<int>(waypoints.wpnts.size()) - 1);
}

double Controller::get_curvature_at_index(int index) {
    if (!has_velocity_path_ || current_waypoints_.wpnts.empty()) {
        return 0.0;  // Default to no curvature if no waypoint data available
    }
    
    // Clamp index to valid range
    int valid_index = std::max(0, std::min(index, static_cast<int>(current_waypoints_.wpnts.size()) - 1));

    if (valid_index >= static_cast<int>(current_waypoints_.wpnts.size())) {
        return 0.0;
    }
    
    return current_waypoints_.wpnts[valid_index].kappa_radpm;
}

} // namespace controller_pkg

// Global pointer to the node for signal handler
std::shared_ptr<controller_pkg::Controller> g_node = nullptr;

void signalHandler(int /* signum */) {
    if (g_node) {
        g_node->shutdown_handler();
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<controller_pkg::Controller>();
    g_node = node;  // Set global pointer for signal handler
    
    // Register signal handlers
    std::signal(SIGINT, signalHandler);   // Ctrl+C
    std::signal(SIGTERM, signalHandler);  // Termination signal
    
    RCLCPP_INFO(node->get_logger(), "Starting Controller Node with graceful shutdown");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}