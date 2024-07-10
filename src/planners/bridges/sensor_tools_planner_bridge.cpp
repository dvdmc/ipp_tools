#include "ipp_tools/planners/bridges/sensors_tools_planner_bridge.h"

#include "sensors_tools_ros/MoveSensor.h"

namespace ipp_tools {
namespace planners {
namespace bridges {

SensorsToolsPlannerBridge::SensorsToolsPlannerBridge(ros::NodeHandle &nh) : nh_(nh) {
  plan_srv_ = nh_.serviceClient<sensors_tools_ros::MoveSensor>("move_sensor");
}

SensorsToolsPlannerBridge::~SensorsToolsPlannerBridge() {}

bool SensorsToolsPlannerBridge::sendPose(geometry_msgs::PoseStamped pose) {
  Eigen::Vector3f current_translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  Eigen::Quaternionf current_rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
  current_goal_ = Eigen::Affine3f::Identity();
  current_goal_.translation() = current_translation;
  current_goal_.linear() = current_rotation.toRotationMatrix();

  sensors_tools_ros::MoveSensor::Request req;
  req.pose = pose;
  sensors_tools_ros::MoveSensor::Response res;
  plan_srv_.call(req, res);

  return true;
}

bool SensorsToolsPlannerBridge::sendPose(Eigen::Affine3f eigen_pose) {
  
    current_goal_ = eigen_pose;
  
    sensors_tools_ros::MoveSensor::Request req;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map"; // TODO: This is assumed. Fix in the future.
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = eigen_pose.translation().x();
    pose_msg.pose.position.y = eigen_pose.translation().y();
    pose_msg.pose.position.z = eigen_pose.translation().z();
    Eigen::Quaternionf rotation(eigen_pose.linear());
    pose_msg.pose.orientation.w = rotation.w();
    pose_msg.pose.orientation.x = rotation.x();
    pose_msg.pose.orientation.y = rotation.y();
    pose_msg.pose.orientation.z = rotation.z();
    req.pose = pose_msg;

    sensors_tools_ros::MoveSensor::Response res;
    plan_srv_.call(req, res);
  
    return true;
}

bool SensorsToolsPlannerBridge::goalReached(Eigen::Affine3f current_pose) {
  float dist = (current_goal_.translation() - current_pose.translation()).norm();
  float angle = (current_goal_.linear() * current_pose.linear().inverse()).trace();

  return dist < dist_threshold_ && angle > angle_threshold_;
}

bool SensorsToolsPlannerBridge::isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) {
  return true;
}

BridgeStatus SensorsToolsPlannerBridge::getStatus() {
  // Currently, the functionality is quite basic
  return BridgeStatus::IDLE;
}



}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools