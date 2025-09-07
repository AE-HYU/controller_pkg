#pragma once

#include <nav_msgs/msg/path.hpp>
#include <cmath>
#include <vector>

namespace controller_pkg {

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    bool valid = false;
};

namespace utils {

// Angle normalization utility
double normalize_angle(double angle);

// Path utilities
double distance_to_path(const nav_msgs::msg::Path& path, const VehicleState& vehicle);
bool is_path_valid(const nav_msgs::msg::Path& path);
double calculate_path_distance_at_index(const nav_msgs::msg::Path& path, int index);

} // namespace utils
} // namespace controller_pkg