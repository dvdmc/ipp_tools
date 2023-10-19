#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include "ipp_tools_ros/ros_utils.hpp"
#include "ipp_tools_ros/rrt_nav2_interface.hpp"

namespace ipp_tools
{
namespace nav2_interface
{

void RRTBasic::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Create map from costmap

  map_ = std::make_shared<maps::Costmap2D>(costmap_ros_);

  // TODO: this is probably wrong
  common::Limits2D limits(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(costmap_ros_->getSizeInMetersX(), costmap_ros_->getSizeInMetersY(), -M_PI));
  
  // Create planner
  planner_ = std::make_unique<global_planner::RRT<Eigen::Affine2d, common::Limits2D>>(map_, limits);
}

void RRTBasic::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTBasic::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTBasic::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path RRTBasic::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  // Transform the start and goal poses to Affine2d and pass to RRT planner
  Eigen::Affine2d start_pose, goal_pose;
  tf2::fromMsg(start.pose, start_pose);
  tf2::fromMsg(goal.pose, goal_pose);
  
  global_path.poses.clear();
  global_path.header.frame_id = global_frame_;

  std::vector<Eigen::Affine2d> path;
  // calculating the number of loops for current value of interpolation_resolution_
  planner_->createPlan(start_pose, goal_pose, path);

  // Fill the path with the result
  for (auto & pose : path) {
    geometry_msgs::msg::PoseStamped pose_stamped = ros_utils::affineToPoseStamped<Eigen::Affine2d>(pose);
    pose_stamped.header.frame_id = global_frame_;
    pose_stamped.header.stamp = node_->now();

    global_path.poses.push_back(pose_stamped);
  }

  global_path.header.stamp = node_->now();

  return global_path;
}

}  // namespace nav2_interface
}  // namespace ipp_tools

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ipp_tools::nav2_interface::RRTBasic, nav2_core::GlobalPlanner)