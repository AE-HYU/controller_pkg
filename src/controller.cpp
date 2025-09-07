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

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized");

    // Initialize publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10);

    
    // Initialize subscribers (only path_with_velocity and odom)
    path_with_velocity_sub_ = this->create_subscription<planning_custom_msgs::msg::PathWithVelocity>(
        "/planned_path_with_velocity", 10,
        std::bind(&Controller::path_with_velocity_callback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&Controller::odom_callback, this, std::placeholders::_1));
    
    // Initialize control timer (50 Hz)
    auto timer_period = std::chrono::milliseconds(20);
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&Controller::control_timer_callback, this));
    
    last_path_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Controller subscribed to /planned_path_with_velocity topic");
    
    return true;
}

void Controller::path_with_velocity_callback(const planning_custom_msgs::msg::PathWithVelocity::SharedPtr msg) {
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty velocity path");
        return;
    }
    
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    // Store the velocity path
    current_velocity_path_ = *msg;
    has_velocity_path_ = true;
    
    // Convert PathWithVelocity to nav_msgs::Path for compatibility with existing algorithms
    current_path_.header = msg->header;
    current_path_.poses.clear();
    current_path_.poses.reserve(msg->points.size());
    
    for (const auto& point : msg->points) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose.position.x = point.x;
        pose_stamped.pose.position.y = point.y;
        pose_stamped.pose.position.z = 0.0;
        
        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, point.yaw);
        pose_stamped.pose.orientation = tf2::toMsg(q);
        
        current_path_.poses.push_back(pose_stamped);
    }
    
    path_received_ = true;
    last_path_time_ = this->get_clock()->now();
    last_target_index_ = 0;  // Reset target index for new path
    
    RCLCPP_INFO(this->get_logger(), "Received and converted velocity path with %zu points", 
                 current_velocity_path_.points.size());
}

void Controller::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    vehicle_state_.x = msg->pose.pose.position.x;
    vehicle_state_.y = msg->pose.pose.position.y;
    
    // Convert quaternion to yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    vehicle_state_.yaw = yaw;
    
    // Calculate velocity
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    vehicle_state_.velocity = std::sqrt(vx*vx + vy*vy);
    // std::cout << "vehicle_state : " << vehicle_state_.velocity << std::endl;
    vehicle_state_.valid = true;
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
    nav_msgs::msg::Path path;
    VehicleState vehicle;
    
    {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        path = current_path_;
        vehicle = vehicle_state_;
    }
    
    if (path.poses.empty()) {
        return std::make_pair(0.0, 0.0);
    }
    
    int target_index = find_target_point(path, vehicle);
    if (target_index < 0 || target_index >= static_cast<int>(path.poses.size())) {
        RCLCPP_WARN(this->get_logger(), "No valid target point found");
        return std::make_pair(0.0, 0.0);
    }
    // Get target point coordinates
    double target_x = path.poses[target_index].pose.position.x;
    double target_y = path.poses[target_index].pose.position.y;
    // Extract curvature information (only if velocity path is available)
    double curvature = 0.0;
    if (has_velocity_path_ && target_index >= 0 && target_index < static_cast<int>(current_velocity_path_.points.size())) {
        curvature = current_velocity_path_.points[target_index].curvature;
    }
    // Calculate steering angle
    double steering_angle = calculate_steering_angle(target_x, target_y, vehicle);
    // Get velocity from planner
    double speed = get_target_speed(target_index);
    RCLCPP_DEBUG(this->get_logger(), "Control: target=%d, steering=%.2f°, speed=%.1f, curvature=%.3f", 
                 target_index, steering_angle*180/M_PI, speed, curvature);
    // Clamp values
    steering_angle = std::clamp(steering_angle, -config_.max_steering_angle, 
                               config_.max_steering_angle);
    speed = std::clamp(speed, config_.min_speed, config_.max_speed);
    return std::make_pair(steering_angle, speed);
}


std::pair<int, double> Controller::find_vehicle_position_on_path(const nav_msgs::msg::Path& path, 
                                                                  const VehicleState& vehicle) {
    if (path.poses.empty()) {
        return std::make_pair(-1, 0.0);
    }
    
    // Find closest point first
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = 0;
    
    for (int i = 0; i < static_cast<int>(path.poses.size()); ++i) {
        double dx = path.poses[i].pose.position.x - vehicle.x;
        double dy = path.poses[i].pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    // Calculate vehicle's s-coordinate by projecting onto the path segment
    double vehicle_s = utils::calculate_path_distance_at_index(path, closest_index);
    
    if (closest_index < static_cast<int>(path.poses.size()) - 1) {
        // Project vehicle position onto the line segment between closest and next point
        double x1 = path.poses[closest_index].pose.position.x;
        double y1 = path.poses[closest_index].pose.position.y;
        double x2 = path.poses[closest_index + 1].pose.position.x;
        double y2 = path.poses[closest_index + 1].pose.position.y;
        
        double path_dx = x2 - x1;
        double path_dy = y2 - y1;
        double segment_length_sq = path_dx*path_dx + path_dy*path_dy;
        
        if (segment_length_sq > 1e-6) {
            double vehicle_dx = vehicle.x - x1;
            double vehicle_dy = vehicle.y - y1;
            
            // Project onto segment (t = 0 at closest_index, t = 1 at next point)
            double t = (vehicle_dx * path_dx + vehicle_dy * path_dy) / segment_length_sq;
            t = std::clamp(t, 0.0, 1.0);
            
            // Add the projected distance to the cumulative distance
            vehicle_s += t * std::sqrt(segment_length_sq);
        }
    }
    
    return std::make_pair(closest_index, vehicle_s);
}

int Controller::find_target_point(const nav_msgs::msg::Path& path, 
                                   const VehicleState& vehicle) {
    if (path.poses.empty()) return -1;
    
    // To use the curvature of the target point, target_index should be found first and then lookahead calculated.
    // The previous approach calculated lookahead first, but now it is curvature-based.
    // For now, keep the previous approach; curvature will be passed from pure_pursuit_control.
    double lookahead = calculate_lookahead_distance(vehicle.velocity, 0.0); // Curvature will be passed from pure_pursuit_control
    
    // Get vehicle's position along the path (Frenet s-coordinate)
    auto [closest_index, vehicle_s] = find_vehicle_position_on_path(path, vehicle);
    if (closest_index < 0) return -1;
    
    // Target s-coordinate is vehicle's s + lookahead distance
    double target_s = vehicle_s + lookahead;
    
    // Start search from closest index to avoid going backwards
    int start_index = std::max(0, closest_index);
    
    // Find the point along the path that corresponds to target_s
    double cumulative_s = utils::calculate_path_distance_at_index(path, start_index);
    
    for (int i = start_index; i < static_cast<int>(path.poses.size()) - 1; ++i) {
        double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        double segment_length = std::sqrt(dx*dx + dy*dy);
        
        if (cumulative_s + segment_length >= target_s) {
            // Found the segment containing our target s-coordinate
            last_target_index_ = i + 1;
            return i + 1;
        }
        
        cumulative_s += segment_length;
    }
    
    // If target_s is beyond the path, return the last point
    last_target_index_ = static_cast<int>(path.poses.size()) - 1;
    return last_target_index_;
}

/////////////////////////////////////////////////////////////
double Controller::calculate_lookahead_distance(double velocity, double curvature) {
    double base_lookahead = velocity * config_.lookahead_ratio;
    // Reduce lookahead distance as curvature increases (K: tuning parameter)
    double K = 2.0; // Curvature sensitivity, can be parameterized if needed
    double curvature_factor = std::max(1.0, std::abs(curvature) * K);
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
        target_index >= static_cast<int>(current_velocity_path_.points.size())) {
        return -1.0; // Invalid velocity
    }
    
    return current_velocity_path_.points[target_index].velocity;
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