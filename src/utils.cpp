#include "controller_pkg/utils.hpp"
#include <limits>

namespace controller_pkg {
namespace utils {

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double distance_to_path(const nav_msgs::msg::Path& path, const VehicleState& vehicle) {
    if (path.poses.empty()) return 0.0;
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& pose : path.poses) {
        double dx = pose.pose.position.x - vehicle.x;
        double dy = pose.pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

bool is_path_valid(const nav_msgs::msg::Path& path) {
    if (path.poses.empty()) {
        return false;
    }
    
    if (path.poses.size() < 2) {
        return false;  // Need at least 2 points
    }
    
    return true;
}

double calculate_path_distance_at_index(const nav_msgs::msg::Path& path, int index) {
    if (index <= 0 || path.poses.empty()) return 0.0;
    
    double cumulative_distance = 0.0;
    
    for (int i = 1; i <= index && i < static_cast<int>(path.poses.size()); ++i) {
        double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        cumulative_distance += std::sqrt(dx*dx + dy*dy);
    }
    
    return cumulative_distance;
}


} // namespace utils
} // namespace controller_pkg