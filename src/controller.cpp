#include "controller_pkg/controller.hpp"
#include <tf2/utils.h>  // Add this include for tf2::toMsg
#include <csignal>
#include <memory>

namespace controller_pkg {

Controller::Controller() 
    : Node("controller"),
      path_received_(false),
      emergency_stop_(false),
      last_target_index_(0),
      has_velocity_path_(false),
      smoothed_velocity_(0.0),
      previous_steering_angle_(0.0),  // Initialize steering smoothing
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
    this->declare_parameter("corner_detection_angle", config_.corner_detection_angle);
    this->declare_parameter("corner_speed_factor", config_.corner_speed_factor);
    this->declare_parameter("anticipation_distance", config_.anticipation_distance);
    this->declare_parameter("use_planner_velocity", config_.use_planner_velocity);
    this->declare_parameter("velocity_scale_factor", config_.velocity_scale_factor);
    this->declare_parameter("velocity_smoothing", config_.velocity_smoothing);
    this->declare_parameter("stanley_k_e", config_.stanley_k_e);
    this->declare_parameter("stanley_k_soft", config_.stanley_k_soft);
    this->declare_parameter("stanley_heading_gain", config_.stanley_heading_gain);
    this->declare_parameter("stanley_max_cross_track", config_.stanley_max_cross_track);
    this->declare_parameter("stanley_steering_smoothing", config_.stanley_steering_smoothing);
    this->declare_parameter("use_stanley", config_.use_stanley);

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
    config_.corner_detection_angle = this->get_parameter("corner_detection_angle").as_double();
    config_.corner_speed_factor = this->get_parameter("corner_speed_factor").as_double();
    config_.anticipation_distance = this->get_parameter("anticipation_distance").as_double();
    config_.use_planner_velocity = this->get_parameter("use_planner_velocity").as_bool();
    config_.velocity_scale_factor = this->get_parameter("velocity_scale_factor").as_double();
    config_.velocity_smoothing = this->get_parameter("velocity_smoothing").as_double();
    config_.stanley_k_e = this->get_parameter("stanley_k_e").as_double();
    config_.stanley_k_soft = this->get_parameter("stanley_k_soft").as_double();
    config_.stanley_heading_gain = this->get_parameter("stanley_heading_gain").as_double();
    config_.stanley_max_cross_track = this->get_parameter("stanley_max_cross_track").as_double();
    config_.stanley_steering_smoothing = this->get_parameter("stanley_steering_smoothing").as_double();
    config_.use_stanley = this->get_parameter("use_stanley").as_bool();

    // Log which controller is being used
    RCLCPP_INFO(this->get_logger(), "Controller type: %s", 
                config_.use_stanley ? "Stanley" : "Pure Pursuit");

    // Initialize publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10);

    // Initialize Stanley analysis publisher (only if using Stanley controller)
    if (config_.use_stanley) {
        stanley_analysis_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/stanley_analysis", 10);
        RCLCPP_INFO(this->get_logger(), "Stanley analysis data will be published to /stanley_analysis");
    }
    
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
    
    RCLCPP_INFO(this->get_logger(), "Subscribing only to /planned_path_with_velocity topic");
    
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
    
    // Calculate control commands using the configured controller
    double steering_angle, speed;
    
    if (config_.use_stanley) {
        // Use Stanley controller
        auto [stanley_steering, stanley_speed] = stanley_method_control();
        steering_angle = stanley_steering;
        speed = stanley_speed;
        RCLCPP_DEBUG(this->get_logger(), "Using Stanley controller");
    } else {
        // Use Pure Pursuit controller
        auto [pp_steering, pp_speed] = pure_pursuit_control();
        steering_angle = pp_steering;
        speed = pp_speed;
        RCLCPP_DEBUG(this->get_logger(), "Using Pure Pursuit controller");
    }

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
    
    // Detect upcoming corner
    // Reason for corner detection:
    // - Corners require different lookahead and speed handling
    // - Helps to adjust steering and speed dynamically
    bool in_corner = detect_upcoming_corner(path, vehicle);
    
    // Find target point (with shorter lookahead for corners)
    int target_index = find_target_point(path, vehicle, in_corner);
    if (target_index < 0 || target_index >= static_cast<int>(path.poses.size())) {
        RCLCPP_WARN(this->get_logger(), "No valid target point found");
        return std::make_pair(0.0, 0.0);
    }
    
    // Get target point coordinates
    double target_x = path.poses[target_index].pose.position.x;
    double target_y = path.poses[target_index].pose.position.y;
    
    // Calculate steering angle with hybrid approach
    double pure_pursuit_angle = calculate_steering_angle(target_x, target_y, vehicle);
    
    // HYBRID CONTROL: Add cross-track error correction (Stanley-like)
    double cross_track_correction = calculate_cross_track_correction(path, vehicle, target_index);
    double steering_angle = pure_pursuit_angle + cross_track_correction;
    
    RCLCPP_DEBUG(this->get_logger(), "Steering: PP=%.3f, CT=%.3f, Total=%.3f", 
                 pure_pursuit_angle, cross_track_correction, steering_angle);
    
    // Calculate lateral error for speed adjustment
    double lateral_error = utils::distance_to_path(path, vehicle);
    
    // Get velocity from planner if available
    double planner_velocity = get_velocity_at_point(target_index);
    
    // DEBUG: Log path following behavior
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Path Follow: target_idx=%d, lookahead=%.2fm, steering=%.3frad, corner=%s, lat_err=%.2fm", 
        target_index, calculate_lookahead_distance(vehicle.velocity, in_corner), 
        steering_angle, in_corner ? "YES" : "NO", lateral_error);
    
    // Calculate target speed (with corner handling)
    double speed = calculate_target_speed(steering_angle, lateral_error, in_corner, planner_velocity);
    
    if (in_corner) {
        RCLCPP_DEBUG(this->get_logger(), "Corner detected! Reducing speed to %.2f", speed);
    }
    
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
                                   const VehicleState& vehicle, bool in_corner) {
    if (path.poses.empty()) return -1;
    
    double lookahead = calculate_lookahead_distance(vehicle.velocity, in_corner);
    
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
double Controller::calculate_lookahead_distance(double velocity, bool in_corner) {
    double dynamic_lookahead = velocity * config_.lookahead_ratio;
    
    // Slightly reduce lookahead distance in corners for better tracking
    if (in_corner) {
        dynamic_lookahead *= 0.9;  // Small reduction, not aggressive
        RCLCPP_DEBUG(this->get_logger(), "Corner detected: adjusted lookahead %.2fm", dynamic_lookahead);
    }
    
    return std::clamp(dynamic_lookahead, config_.min_lookahead, config_.max_lookahead);
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
    
    // Pure pursuit 에서 1/R을 담당. curvature = 1/R = 2y/d^2
    // R : 뒤축에서 ICR까지의 거리 = lookahead point에서 ICR까지의 거리
    // d : lookahead point와 뒤축 사이의 거리
    // y : local 좌표계의 y축 기준 lookahead point와 뒤축 사이의 거리
    double curvature = 2.0 * local_y / (lookahead_distance * lookahead_distance);
    double steering_angle = std::atan(config_.wheelbase * curvature);
    
    return steering_angle;
}

double Controller::calculate_cross_track_correction(const nav_msgs::msg::Path& path, const VehicleState& vehicle, int target_index) {
    if (path.poses.empty() || target_index < 0 || target_index >= static_cast<int>(path.poses.size())) {
        return 0.0;
    }
    
    // Find closest point for cross-track error calculation
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = 0;
    double cross_track_error = 0.0;
    
    // Look in a small window around target index for efficiency
    int start_idx = std::max(0, target_index - 5);
    int end_idx = std::min(static_cast<int>(path.poses.size()) - 1, target_index + 5);
    
    for (int i = start_idx; i <= end_idx; ++i) {
        double dx = path.poses[i].pose.position.x - vehicle.x;
        double dy = path.poses[i].pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    if (closest_index < static_cast<int>(path.poses.size()) - 1) {
        // Get path direction at closest point
        double path_x1 = path.poses[closest_index].pose.position.x;
        double path_y1 = path.poses[closest_index].pose.position.y;
        double path_x2 = path.poses[closest_index + 1].pose.position.x;
        double path_y2 = path.poses[closest_index + 1].pose.position.y;
        
        double path_dx = path_x2 - path_x1;
        double path_dy = path_y2 - path_y1;
        double path_length = std::sqrt(path_dx*path_dx + path_dy*path_dy);
        
        if (path_length > 1e-6) {
            // Normalize path direction
            path_dx /= path_length;
            path_dy /= path_length;
            
            // Calculate cross-track error (signed distance)
            double vehicle_to_path_x = vehicle.x - path_x1;
            double vehicle_to_path_y = vehicle.y - path_y1;
            
            // Cross product gives signed distance
            cross_track_error = vehicle_to_path_x * (-path_dy) + vehicle_to_path_y * path_dx;
            
            // Stanley-like correction gain (proportional to cross-track error)
            double k_cross_track = 0.5; // Tuning parameter
            double correction = -k_cross_track * std::atan(cross_track_error / (vehicle.velocity + 0.1));
            
            // Limit correction to prevent oscillation
            correction = std::clamp(correction, -0.2, 0.2); // ±0.2 rad limit
            
            RCLCPP_DEBUG(this->get_logger(), "Cross-track: error=%.3fm, correction=%.3frad", 
                         cross_track_error, correction);
            
            return correction;
        }
    }
    
    return 0.0;
}

double Controller::calculate_target_speed(double steering_angle, double lateral_error, bool in_corner, double planner_velocity) {
    double base_speed;
    
    // Use planner velocity if available and enabled
    if (config_.use_planner_velocity && planner_velocity > 0.0) {
        // Scale and smooth planner velocity
        double scaled_velocity = planner_velocity * config_.velocity_scale_factor;
        
        // Smooth velocity changes
        if (smoothed_velocity_ > 0.0) {
            smoothed_velocity_ = smoothed_velocity_ * (1.0 - config_.velocity_smoothing) + 
                               scaled_velocity * config_.velocity_smoothing;
        } else {
            smoothed_velocity_ = scaled_velocity;
        }
        
        base_speed = smoothed_velocity_;
        
        RCLCPP_DEBUG(this->get_logger(), "Using planner velocity: raw=%.2f, scaled=%.2f, smoothed=%.2f", 
                     planner_velocity, scaled_velocity, base_speed);
    } else {
        // Fallback to configured target speed
        base_speed = config_.target_speed;
        smoothed_velocity_ = base_speed;  // Update smoothed velocity for consistency
    }
    
    // Base speed reduction based on steering angle (less aggressive when using planner velocity)
    double abs_steering = std::abs(steering_angle);
    double steering_factor = config_.use_planner_velocity && planner_velocity > 0.0 ? 
                            1.0 - (abs_steering / config_.max_steering_angle) * 0.3 :  // Less aggressive with planner
                            1.0 - (abs_steering / config_.max_steering_angle) * 0.8;   // More aggressive fallback
    
    // Reduce speed based on lateral error
    double error_factor = 1.0 - std::min(lateral_error / config_.max_lateral_error, 1.0) * 0.4;
    
    double target_speed = base_speed * steering_factor * error_factor;
    
    // Apply corner speed reduction
    if (in_corner) {
        target_speed *= config_.corner_speed_factor;  // Very aggressive corner slowdown
    }
    
    return std::max(target_speed, config_.min_speed);
}


double Controller::get_velocity_at_point(int target_index) const {
    if (!has_velocity_path_ || target_index < 0 || 
        target_index >= static_cast<int>(current_velocity_path_.points.size())) {
        return -1.0; // Invalid velocity
    }
    
    return current_velocity_path_.points[target_index].velocity;
}

bool Controller::detect_upcoming_corner(const nav_msgs::msg::Path& path, const VehicleState& vehicle) {
    if (path.poses.size() < 5) {
        return false;
    }
    
    // Find the index of the closest point on the path
    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = 0;
    
    for (int i = 0; i < static_cast<int>(path.poses.size()); ++i) {
        double dx = path.poses[i].pose.position.x - vehicle.x;
        double dy = path.poses[i].pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    // Look ahead on the path to detect corners
    // What is anticipation points?
    // - Anticipation points are used to look ahead on the path to detect corners
    // - They help in calculating curvature and steering requirements for upcoming turns
    // - This allows the vehicle to adjust its speed and steering in advance
    // - Anticipation points are calculated based on the configured anticipation distance
    // - They are used to sample the path ahead of the current position
    int anticipation_points = static_cast<int>(config_.anticipation_distance / 0.2);  // Assuming 0.2m spacing
    int check_start = std::min(closest_idx, static_cast<int>(path.poses.size()) - 5);
    int check_end = std::min(check_start + anticipation_points, static_cast<int>(path.poses.size()) - 1);
    
    // Use planner-provided curvature data (more accurate than recalculating)
    double max_curvature = 0.0;
    if (has_velocity_path_) {
        int velocity_check_start = std::max(0, std::min(check_start, static_cast<int>(current_velocity_path_.points.size()) - 1));
        int velocity_check_end = std::max(0, std::min(check_end, static_cast<int>(current_velocity_path_.points.size()) - 1));
        
        for (int i = velocity_check_start; i <= velocity_check_end; ++i) {
            if (i < static_cast<int>(current_velocity_path_.points.size())) {
                max_curvature = std::max(max_curvature, std::abs(current_velocity_path_.points[i].curvature));
            }
        }
    } else {
        // Fallback: use simple approximation if planner data is not available
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Planner velocity path not available, using fallback curvature detection");
        max_curvature = 0.1;  // Conservative estimate for corner detection
    }
    
    // Also check current steering requirement
    // Predict required steering about the lookahead point
    // Lookahead point : configured anticipation distance along the path
    double predicted_steering = predict_required_steering(path, vehicle);
    
    // Corner detected if either curvature is high or predicted steering is large
    bool high_curvature = max_curvature > 0.5;  // 0.5 rad/m curvature threshold
    bool high_steering = std::abs(predicted_steering) > config_.corner_detection_angle;
    
    return high_curvature || high_steering;
}


double Controller::predict_required_steering(const nav_msgs::msg::Path& path, const VehicleState& vehicle) {
    if (path.poses.size() < 2) {
        return 0.0;
    }
    
    // Use anticipation distance along the path (Frenet s-coordinate)
    double lookahead_s = config_.anticipation_distance;
    
    // Get vehicle's position along the path
    auto [closest_index, vehicle_s] = find_vehicle_position_on_path(path, vehicle);
    if (closest_index < 0) return 0.0;
    
    // Find point at target s-coordinate
    double target_s = vehicle_s + lookahead_s;
    double cumulative_s = utils::calculate_path_distance_at_index(path, closest_index);
    
    for (int i = closest_index; i < static_cast<int>(path.poses.size()) - 1; ++i) {
        double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        double segment_length = std::sqrt(dx*dx + dy*dy);
        
        if (cumulative_s + segment_length >= target_s) {
            // Found the target point, calculate required steering
            double target_x = path.poses[i+1].pose.position.x;
            double target_y = path.poses[i+1].pose.position.y;
            
            double dx_to_target = target_x - vehicle.x;
            double dy_to_target = target_y - vehicle.y;
            
            // Transform to vehicle coordinate frame
            double cos_yaw = std::cos(-vehicle.yaw);
            double sin_yaw = std::sin(-vehicle.yaw);
            double local_x = dx_to_target * cos_yaw - dy_to_target * sin_yaw;
            double local_y = dx_to_target * sin_yaw + dy_to_target * cos_yaw;
            
            if (local_x > 0.1) {  // Avoid division by zero
                double distance = std::sqrt(local_x*local_x + local_y*local_y);
                double curvature = 2.0 * local_y / (distance * distance);
                return std::atan(config_.wheelbase * curvature);
            }
            break;
        }
        
        cumulative_s += segment_length;
    }
    
    return 0.0;
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

void Controller::publish_stanley_analysis_data(double vehicle_yaw, double path_heading, double heading_error,
                                                double cross_track_term, double steering_angle, 
                                                double steering_smoothing, double lateral_error) {
    if (!stanley_analysis_pub_) {
        return; // Publisher not initialized
    }
    
    auto msg = std_msgs::msg::Float64MultiArray();
    
    // Set labels for PlotJuggler reference
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "stanley_analysis";
    msg.layout.dim[0].size = 7;
    msg.layout.dim[0].stride = 7;
    
    // Data order for PlotJuggler:
    // Index 0: vehicle_yaw (rad)
    // Index 1: path_heading (rad)  
    // Index 2: heading_error (rad)
    // Index 3: cross_track_term (rad)
    // Index 4: steering_angle (rad)
    // Index 5: steering_smoothing (factor)
    // Index 6: lateral_error (m)
    msg.data = {
        vehicle_yaw,
        path_heading, 
        heading_error,
        cross_track_term,
        steering_angle,
        steering_smoothing,
        lateral_error
    };
    
    stanley_analysis_pub_->publish(msg);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Add the Stanley controller methods here, within the namespace and with proper class scope
std::pair<double, double> Controller::stanley_method_control() {
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
    
    // Find closest point on path compared with front_axle position and calculate cross-track error
    auto [closest_point, closest_index, cross_track_error] = find_closest_point_on_path(path, vehicle);
    
    // Calculate path heading at closest point
    double path_heading = calculate_path_heading_at_point(path, closest_index);
    
    // Calculate heading error
    double heading_error = utils::normalize_angle(path_heading - vehicle.yaw);
    
    // Stanley controller parameters from configuration
    double k_e = config_.stanley_k_e;        // Cross-track error gain
    double k_soft = config_.stanley_k_soft;  // Softening constant
    double heading_gain = config_.stanley_heading_gain;  // Heading error gain
    
    // Stanley formula: δ = heading_gain * ψ + arctan(k_e * e / (k_soft + v))
    double velocity = std::max(vehicle.velocity, 0.5); // Minimum velocity for stability
    double cross_track_term = std::atan(k_e * cross_track_error / (k_soft + velocity));
    
    // Limit cross-track correction to prevent overshoot
    cross_track_term = std::clamp(cross_track_term, -config_.stanley_max_cross_track, 
                                  config_.stanley_max_cross_track);
    
    double steering_angle = heading_gain * heading_error + cross_track_term;
    
    // // Apply steering smoothing to reduce oscillations (low-pass filter)
    // double steering_smoothing = config_.stanley_steering_smoothing; // Use configurable smoothing factor
    // steering_angle = previous_steering_angle_ * steering_smoothing + 
    //                  steering_angle * (1.0 - steering_smoothing);
    
    // Update previous steering angle for next iteration
    // previous_steering_angle_ = steering_angle;
    
    // Detect corners for speed adjustment
    // bool in_corner = detect_upcoming_corner(path, vehicle);
    
    // Calculate lateral error for speed adjustment
    double lateral_error = std::abs(cross_track_error);
    
    // Get velocity from planner if available
    int target_index = find_target_point_for_velocity(path, vehicle);
    double planner_velocity = get_velocity_at_point(target_index);
    
    // Calculate target speed
    // double speed = calculate_target_speed(steering_angle, lateral_error, in_corner, planner_velocity);
    double speed = planner_velocity;
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                 "Stanley: heading_err=%.3f, cross_track_err=%.3f, cross_track_term=%.3f, total=%.3f",
                 heading_error*180/M_PI, cross_track_error, cross_track_term*180/M_PI, steering_angle*180/M_PI);
    
    // Clamp values
    steering_angle = std::clamp(steering_angle, -config_.max_steering_angle, 
                               config_.max_steering_angle);
    speed = std::clamp(speed, config_.min_speed, config_.max_speed);

    // Publish analysis data for PlotJuggler
    publish_stanley_analysis_data(vehicle.yaw, path_heading, heading_error, 
                                 cross_track_term, steering_angle, 
                                 config_.stanley_steering_smoothing, lateral_error);
    
    return std::make_pair(steering_angle, speed);
}

std::tuple<geometry_msgs::msg::Point, int, double> 
Controller::find_closest_point_on_path(const nav_msgs::msg::Path& path, 
                                         const VehicleState& vehicle) {
    geometry_msgs::msg::Point closest_point;
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = 0;
    double cross_track_error = 0.0;
    
    // Calculate front axle position from base_link
    double front_axle_x = vehicle.x + config_.wheelbase * std::cos(vehicle.yaw);
    double front_axle_y = vehicle.y + config_.wheelbase * std::sin(vehicle.yaw);
    
    // Find the closest point on the path using front axle position
    for (int i = 0; i < static_cast<int>(path.poses.size()); ++i) {
        double dx = path.poses[i].pose.position.x - front_axle_x;  // Use front axle
        double dy = path.poses[i].pose.position.y - front_axle_y;  // Use front axle
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
            closest_point = path.poses[i].pose.position;
        }
    }
    
    // Calculate cross-track error using line segment projection
    if (closest_index < static_cast<int>(path.poses.size()) - 1) {
        // Get line segment from closest point to next point
        double x1 = path.poses[closest_index].pose.position.x;
        double y1 = path.poses[closest_index].pose.position.y;
        double x2 = path.poses[closest_index + 1].pose.position.x;
        double y2 = path.poses[closest_index + 1].pose.position.y;
        
        // Vector from point 1 to point 2
        double path_dx = x2 - x1;
        double path_dy = y2 - y1;
        double path_length_sq = path_dx*path_dx + path_dy*path_dy;
        
        if (path_length_sq > 1e-6) {
            // Vector from point 1 to front axle (not vehicle base_link)
            double front_axle_dx = front_axle_x - x1;
            double front_axle_dy = front_axle_y - y1;
            
            // Project front axle position onto the line segment
            double t = (front_axle_dx * path_dx + front_axle_dy * path_dy) / path_length_sq;
            t = std::clamp(t, 0.0, 1.0); // Clamp to line segment
            
            // Find projection point
            double proj_x = x1 + t * path_dx;
            double proj_y = y1 + t * path_dy;
            
            // Calculate signed cross-track error from front axle (positive = left of path)
            double to_front_axle_x = front_axle_x - proj_x;
            double to_front_axle_y = front_axle_y - proj_y;
            
            // Normalize path direction
            double path_length = std::sqrt(path_length_sq);
            double path_unit_x = path_dx / path_length;
            double path_unit_y = path_dy / path_length;
            
            // Cross product gives signed distance (right-hand rule)
            cross_track_error = to_front_axle_x * (-path_unit_y) + to_front_axle_y * path_unit_x;
            
            // Update closest point to projection
            closest_point.x = proj_x;
            closest_point.y = proj_y;
        }
    }
    
    return std::make_tuple(closest_point, closest_index, cross_track_error);
}

double Controller::calculate_path_heading_at_point(const nav_msgs::msg::Path& path, int index) {
    if (path.poses.empty() || index < 0 || index >= static_cast<int>(path.poses.size())) {
        return 0.0;
    }
    
    // Use forward difference if possible, otherwise backward difference
    int next_index = (index < static_cast<int>(path.poses.size()) - 1) ? index + 1 : index - 1;
    if (next_index < 0 || next_index >= static_cast<int>(path.poses.size())) {
        return 0.0;
    }
    
    double dx = path.poses[next_index].pose.position.x - path.poses[index].pose.position.x;
    double dy = path.poses[next_index].pose.position.y - path.poses[index].pose.position.y;
    
    // If using backward difference, reverse the direction
    if (next_index < index) {
        dx = -dx;
        dy = -dy;
    }
    
    return std::atan2(dy, dx);
}

int Controller::find_target_point_for_velocity(const nav_msgs::msg::Path& path, 
                                                 const VehicleState& vehicle) {
    if (path.poses.empty()) return -1;
    
    // Use a small lookahead for velocity lookup (1m along path)
    double lookahead_s = 1.0;
    
    // Get vehicle's position along the path
    auto [closest_index, vehicle_s] = find_vehicle_position_on_path(path, vehicle);
    if (closest_index < 0) return -1;
    
    // Target s-coordinate for velocity lookup
    double target_s = vehicle_s + lookahead_s;
    
    // Find the point that corresponds to target_s
    double cumulative_s = utils::calculate_path_distance_at_index(path, closest_index);
    
    for (int i = closest_index; i < static_cast<int>(path.poses.size()) - 1; ++i) {
        double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        double segment_length = std::sqrt(dx*dx + dy*dy);
        
        if (cumulative_s + segment_length >= target_s) {
            return i + 1;
        }
        
        cumulative_s += segment_length;
    }
    
    // If target_s is beyond the path, return the last point
    return static_cast<int>(path.poses.size()) - 1;
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