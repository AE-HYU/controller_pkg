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

double calculate_path_curvature(const nav_msgs::msg::Path& path, int start_idx, int samples) {
    if (path.poses.size() < 3 || start_idx < 0 || 
        start_idx + samples >= static_cast<int>(path.poses.size())) {
        return 0.0;
    }
    
    double max_curvature = 0.0;
    
    for (int i = start_idx + 1; i < start_idx + samples - 1 && i < static_cast<int>(path.poses.size()) - 1; ++i) {
        // Calculate curvature using three consecutive points
        double x1 = path.poses[i-1].pose.position.x;
        double y1 = path.poses[i-1].pose.position.y;
        double x2 = path.poses[i].pose.position.x;
        double y2 = path.poses[i].pose.position.y;
        double x3 = path.poses[i+1].pose.position.x;
        double y3 = path.poses[i+1].pose.position.y;
        
        // Calculate cross product for curvature
        double dx1 = x2 - x1;
        double dy1 = y2 - y1;
        double dx2 = x3 - x2;
        double dy2 = y3 - y2;
        
        double cross_product = std::abs(dx1 * dy2 - dy1 * dx2);
        double magnitude1 = std::sqrt(dx1*dx1 + dy1*dy1);
        double magnitude2 = std::sqrt(dx2*dx2 + dy2*dy2);
        
        if (magnitude1 > 0.01 && magnitude2 > 0.01) {
            double curvature = cross_product / (magnitude1 * magnitude2 * (magnitude1 + magnitude2));
            max_curvature = std::max(max_curvature, curvature);
        }
    }
    
    return max_curvature;
}

} // namespace utils
} // namespace controller_pkg