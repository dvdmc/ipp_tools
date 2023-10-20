#include <cmath>
#include <string>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

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

  // Get map limits
  Eigen::Vector2d origin(costmap_ros_->getOriginX(), costmap_ros_->getOriginY());
  Eigen::Vector2d size(costmap_ros_->getSizeInMetersX(), costmap_ros_->getSizeInMetersY());
  Eigen::Vector2d min = origin - size / 2;
  Eigen::Vector2d max = origin + size / 2;

  Eigen::Vector3d limits_min = Eigen::Vector3d(min.x(), min.y(), -M_PI);
  Eigen::Vector3d limits_max = Eigen::Vector3d(max.x(), max.y(), M_PI);
  common::Limits2D limits(limits_min, limits_max);

  // Create planner
  planner_ = std::make_unique<global_planner::RRT<Eigen::Affine2d, common::Limits2D>>(map_, limits);

  // Create visualizer
  visualizer_ = std::make_unique<visualizers::PathVisualizer>(node_);

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
  // All this logic is to reuse the paths. TODO: simplify, it is horrendous
  // If there is a previous path, check if it still works
  if (last_path_.poses.size() > 0) {
    // Check if the goal is the same
    bool is_same_goal = abs(last_path_.poses.back().pose.position.x - goal.pose.position.x) < 0.1 &&
                        abs(last_path_.poses.back().pose.position.y - goal.pose.position.y) < 0.1;

    if(is_same_goal)
    {
      // Check if some of the points in the path can be removed (distance from current to next is less than previous to next)
      const geometry_msgs::msg::PoseStamped* current = &start;
      
      for(int i=0; i<last_path_.poses.size(); i++)
      {
        const geometry_msgs::msg::PoseStamped* waypoint = &last_path_.poses[i];
        const geometry_msgs::msg::PoseStamped* next = &last_path_.poses[i+1];
        // To avoid overcomplicated stuff, if the robot is far from the waypoint, we assume that the path is still valid
        double distance_current_waypoint = std::sqrt(std::pow(current->pose.position.x - waypoint->pose.position.x, 2) + std::pow(current->pose.position.y - waypoint->pose.position.y, 2));
        
        if(distance_current_waypoint > 1.0)
        {
          continue;
        }
        double distance_current_next = std::sqrt(std::pow(current->pose.position.x - next->pose.position.x, 2) + std::pow(current->pose.position.y - next->pose.position.y, 2));
        double distance_previous_next = std::sqrt(std::pow(waypoint->pose.position.x - next->pose.position.x, 2) + std::pow(waypoint->pose.position.y - next->pose.position.y, 2));
        if(distance_current_next < distance_previous_next)
        {
          // Remove the waypoint
          last_path_.poses.erase(last_path_.poses.begin() + i);
        }
      }
      return last_path_;
    }
    
  }

  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  // Transform the start and goal poses to Affine2d and pass to RRT planner
  Eigen::Affine3d start_pose_3d, goal_pose_3d;
  tf2::fromMsg(start.pose, start_pose_3d);
  tf2::fromMsg(goal.pose, goal_pose_3d);
  Eigen::Affine2d start_pose = ros_utils::convertAffine3dToAffine2d(start_pose_3d);
  Eigen::Affine2d goal_pose = ros_utils::convertAffine3dToAffine2d(goal_pose_3d);

  global_path.poses.clear();
  global_path.header.frame_id = global_frame_;

  std::vector<Eigen::Affine2d> path;
  RCLCPP_WARN(node_->get_logger(), "START POSE %f %f", start_pose.translation().x(), start_pose.translation().y());
  RCLCPP_WARN(node_->get_logger(), "GOAL POSE %f %f", goal_pose.translation().x(), goal_pose.translation().y());
  bool success = planner_->createPlan(start_pose, goal_pose, path, 0.1, 5000);
  
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "PATH NOT OBTAINED");
    return global_path;
  } else {
    RCLCPP_WARN(node_->get_logger(), "PATH OBTAINED");
    RCLCPP_WARN(node_->get_logger(), "PATH SIZE %d", path.size());
  }

  // Fill the path with the result
  for (Eigen::Affine2d& pose : path) {
    geometry_msgs::msg::PoseStamped pose_stamped = ros_utils::affineToPoseStamped<Eigen::Affine2d>(pose);
    RCLCPP_WARN(node_->get_logger(), "PATH POSE %f %f", pose_stamped.pose.position.x, pose_stamped.pose.position.y);
    pose_stamped.header.frame_id = global_frame_;
    pose_stamped.header.stamp = node_->now();

    global_path.poses.push_back(pose_stamped);
  }

  // Reverse the path (for some reason... TODO: check why)
  std::reverse(global_path.poses.begin(), global_path.poses.end());

  global_path.header.stamp = node_->now();

  //visualizer_->visualizePath(global_path);

  last_path_ = global_path;

  return global_path;
}

}  // namespace nav2_interface
}  // namespace ipp_tools

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ipp_tools::nav2_interface::RRTBasic, nav2_core::GlobalPlanner)