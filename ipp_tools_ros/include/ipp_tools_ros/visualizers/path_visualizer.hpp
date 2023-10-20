#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ipp_tools {
namespace visualizers {

class PathVisualizer {
public:
    PathVisualizer(rclcpp_lifecycle::LifecycleNode::SharedPtr node) : node_(node) {
        // Initialize the marker publisher
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("path_marker", 10);
    }

    void visualizePath(const nav_msgs::msg::Path& path) {
        // Create a marker for the path
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = path.header.frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.r = 1.0;
        marker.color.a = 1.0;

        // Add the path points to the marker
        for (const auto& pose : path.poses) {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            marker.points.push_back(point);
        }

        // Publish the marker
        marker_pub_->publish(marker);
    }

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

} // namespace visualizers
} // namespace ipp_tools