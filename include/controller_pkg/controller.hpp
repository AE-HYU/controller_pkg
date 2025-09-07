#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <planning_custom_msgs/msg/path_with_velocity.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <atomic>
#include "controller_pkg/utils.hpp"

namespace controller_pkg {

struct ControllerConfig {
    // Pure Pursuit parameters
    double lookahead_distance = 2.5;
    double min_lookahead = 0.5;
    double max_lookahead = 3.0;
    double lookahead_ratio = 0.4;
    
    // Vehicle parameters
    double wheelbase = 0.35;
    double max_steering_angle = 0.4;
    
    // Speed control
    double target_speed = 5.0;
    double max_speed = 8.0;
    double min_speed = 1.0;
    
    // Planner velocity integration
    bool use_planner_velocity = true;
    double velocity_scale_factor = 1.0;
    
    // Curvature-based lookahead adjustment
    double curvature_sensitivity = 2.0;
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
    int find_target_point(const planning_custom_msgs::msg::PathWithVelocity& path, const VehicleState& vehicle);
    double calculate_lookahead_distance(double velocity, double curvature = 0.0);
    double calculate_steering_angle(double target_x, double target_y, const VehicleState& vehicle);
    double get_target_speed(int target_index);
    double get_velocity_at_point(int target_index) const;
    
    // Safety functions
    void publish_stop_command();
    
    // Helper functions
    std::pair<int, double> find_vehicle_position_on_path(const planning_custom_msgs::msg::PathWithVelocity& path, 
                                                        const VehicleState& vehicle);
    int find_point_at_distance(const planning_custom_msgs::msg::PathWithVelocity& path, int start_index, double target_s);
    double get_curvature_at_index(int index);
    
    // Subscriptions and Publishers
    rclcpp::Subscription<planning_custom_msgs::msg::PathWithVelocity>::SharedPtr path_with_velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    planning_custom_msgs::msg::PathWithVelocity current_velocity_path_;
    VehicleState vehicle_state_;
    ControllerConfig config_;
    
    // Thread safety
    std::mutex path_mutex_;
    std::mutex state_mutex_;
    
    // Control variables
    rclcpp::Time last_path_time_;
    bool path_received_;
    bool emergency_stop_;
    bool has_velocity_path_;
    int last_target_index_;
    
    // Shutdown control
    std::atomic<bool> shutdown_requested_;
};

} // namespace controller_pkg