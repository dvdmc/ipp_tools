#include "ipp_tools_ros/ros_utils.hpp"

namespace ipp_tools
{

namespace ros_utils
{

template <typename Xn>
geometry_msgs::msg::PoseStamped affineToPoseStamped(const Xn& affine)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = affine.translation().x();
    pose.pose.position.y = affine.translation().y();
    pose.pose.position.z = 0.0;
    Eigen::Quaterniond q(affine.rotation());
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

} // namespace ros_utils

} // namespace ipp_tools