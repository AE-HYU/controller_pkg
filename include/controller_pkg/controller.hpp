#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <planning_custom_msgs/msg/path_with_velocity.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include "controller_pkg/utils.hpp"

namespace controller_pkg {

struct ControllerConfig {
    // Pure Pursuit parameters
    double lookahead_distance = 2.0;
    double min_lookahead = 1.0;
    double max_lookahead = 5.0;
    double lookahead_ratio = 0.2;  // lookahead = velocity * ratio
    
    // Vehicle parameters
    double wheelbase = 0.33;
    double max_steering_angle = 0.4;  // radians (~23 degrees)
    
    // Speed control
    double target_speed = 3.0;
    double max_speed = 6.0;
    double min_speed = 1.0;
    double speed_lookahead_gain = 0.5;
    
    // Corner handling
    double corner_detection_angle = 0.3;
    double corner_speed_factor = 0.4;
    double anticipation_distance = 2.0;
    
    // Safety parameters
    double max_lateral_error = 2.0;
    double emergency_brake_distance = 0.5;
    
    // Planner velocity integration
    bool use_planner_velocity = true;
    double velocity_scale_factor = 0.8;
    double velocity_smoothing = 0.2;
    
    // Stanley controller parameters
    double stanley_k_e = 1.2;        // Cross-track error gain
    double stanley_k_soft = 0.5;     // Softening constant
    double stanley_heading_gain = 0.8;  // Heading error gain
    double stanley_max_cross_track = 0.15; // Maximum cross-track correction
    double stanley_steering_smoothing = 0.3; // Steering smoothing factor
    bool use_stanley = true;          // Use Stanley controller flag
};

class Controller : public rclcpp::Node {
public:
    Controller();
    virtual ~Controller();
    
    // Public safety functions
    void shutdown_handler();

private:
    bool initialize();
    
    // Callbacks
    void path_with_velocity_callback(const planning_custom_msgs::msg::PathWithVelocity::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_timer_callback();
    
    // Pure Pursuit Controller
    std::pair<double, double> pure_pursuit_control();
    int find_target_point(const nav_msgs::msg::Path& path, const VehicleState& vehicle, bool in_corner = false);
    double calculate_lookahead_distance(double velocity, bool in_corner = false);
    double calculate_steering_angle(double target_x, double target_y, const VehicleState& vehicle);
    double calculate_cross_track_correction(const nav_msgs::msg::Path& path, const VehicleState& vehicle, int target_index);
    double calculate_target_speed(double steering_angle, double lateral_error, bool in_corner = false, double planner_velocity = -1.0);
    
    // Stanley Controller
    std::pair<double, double> stanley_method_control();
    std::tuple<geometry_msgs::msg::Point, int, double> find_closest_point_on_path(const nav_msgs::msg::Path& path, const VehicleState& vehicle);
    double calculate_path_heading_at_point(const nav_msgs::msg::Path& path, int index);
    int find_target_point_for_velocity(const nav_msgs::msg::Path& path, const VehicleState& vehicle);

    // Corner detection and handling
    bool detect_upcoming_corner(const nav_msgs::msg::Path& path, const VehicleState& vehicle);
    double predict_required_steering(const nav_msgs::msg::Path& path, const VehicleState& vehicle);
    double get_velocity_at_point(int target_index) const;
    double get_max_curvature_in_range(int start_idx, int end_idx) const;
    
    // Safety functions
    void publish_stop_command();

    // Analysis functions
    void publish_stanley_analysis_data(double vehicle_yaw, double path_heading, double heading_error,
                                     double cross_track_term, double steering_angle, 
                                     double steering_smoothing, double lateral_error);
    
    
    /**
     * @brief Find vehicle's position along the path in Frenet coordinates
     * @param path Current path
     * @param vehicle Current vehicle state
     * @return Pair of (closest_index, s_coordinate)
     */
    std::pair<int, double> find_vehicle_position_on_path(const nav_msgs::msg::Path& path, 
                                                        const VehicleState& vehicle);
    
    // Subscriptions and Publishers
    // Remove: rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<planning_custom_msgs::msg::PathWithVelocity>::SharedPtr path_with_velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stanley_analysis_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    nav_msgs::msg::Path current_path_;  // Keep this for compatibility with path following algorithms
    planning_custom_msgs::msg::PathWithVelocity current_velocity_path_;
    bool has_velocity_path_;
    double smoothed_velocity_;
    double previous_steering_angle_;
    VehicleState vehicle_state_;
    ControllerConfig config_;
    
    // Thread safety
    std::mutex path_mutex_;
    std::mutex state_mutex_;
    
    // Control variables
    rclcpp::Time last_path_time_;
    bool path_received_;
    bool emergency_stop_;
    int last_target_index_;
    
    // Shutdown control
    std::atomic<bool> shutdown_requested_;
};

} // namespace path_follower_pkg