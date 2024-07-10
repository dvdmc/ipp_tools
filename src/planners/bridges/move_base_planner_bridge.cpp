#include "ipp_tools/planners/bridges/move_base_planner_bridge.h"

#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace ipp_tools {
namespace planners {
namespace bridges {

MoveBasePlannerBridge::MoveBasePlannerBridge(ros::NodeHandle &nh) : nh_(nh), move_base_client_("move_base", true) {
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  move_base_client_.waitForServer();
}

MoveBasePlannerBridge::~MoveBasePlannerBridge() {}

bool MoveBasePlannerBridge::sendPose(geometry_msgs::PoseStamped pose) {
  Eigen::Vector3f current_translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  Eigen::Quaternionf current_rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
  current_goal_ = Eigen::Affine3f::Identity();
  current_goal_.translation() = current_translation;
  current_goal_.linear() = current_rotation.toRotationMatrix();

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = pose.header.frame_id;
  goal.target_pose.header.stamp = pose.header.stamp;
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header = pose.header;
  goal_pose.pose = pose.pose;

  goal.target_pose.pose = goal_pose.pose;
  goal.target_pose.pose.position.z = 0.0;

  move_base_client_.sendGoal(goal);

  return true;
}

bool MoveBasePlannerBridge::sendPose(Eigen::Affine3f eigen_pose) {

  current_goal_ = eigen_pose;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map"; // TODO: This is assumed. Fix in the future.
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = eigen_pose.translation().x();
  goal.target_pose.pose.position.y = eigen_pose.translation().y();
  goal.target_pose.pose.position.z = eigen_pose.translation().z();
  Eigen::Quaternionf rotation(eigen_pose.linear());
  goal.target_pose.pose.orientation.w = rotation.w();
  goal.target_pose.pose.orientation.x = rotation.x();
  goal.target_pose.pose.orientation.y = rotation.y();
  goal.target_pose.pose.orientation.z = rotation.z();

  move_base_client_.sendGoal(goal);

  return true;
}

bool MoveBasePlannerBridge::goalReached(Eigen::Affine3f current_pose) {
  // Use the move_base client to check if the goal has been reached
  std::cout << "MoveBasePlannerBridge check. Current state: " << move_base_client_.getState().toString() << std::endl;
  return move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool MoveBasePlannerBridge::isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) {
  return true;
}

BridgeStatus MoveBasePlannerBridge::getStatus() {
  // Get the status of the move_base client
  actionlib::SimpleClientGoalState state = move_base_client_.getState();
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return BridgeStatus::IDLE;
  } else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
    return BridgeStatus::RUNNING;
  } else if (state == actionlib::SimpleClientGoalState::PENDING) {
    return BridgeStatus::RUNNING;
  } else if (state == actionlib::SimpleClientGoalState::RECALLED) {
    return BridgeStatus::IDLE;
  } else if (state == actionlib::SimpleClientGoalState::REJECTED) {
    return BridgeStatus::ERROR;
  } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
    return BridgeStatus::ERROR;
  } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
    return BridgeStatus::ERROR;
  } else if (state == actionlib::SimpleClientGoalState::LOST) {
    return BridgeStatus::ERROR;
  } else {
    return BridgeStatus::IDLE;
  }
}



}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools