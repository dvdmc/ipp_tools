#include "ipp_tools/planners/bridges/airsim_planner_bridge.h"

namespace ipp_tools {
namespace planners {
namespace bridges {

AirsimPlannerBridge::AirsimPlannerBridge(/* args */) {
  client.confirmConnection();
}

AirsimPlannerBridge::~AirsimPlannerBridge() {}

bool AirsimPlannerBridge::sendPose(Pose pose) {
  client.simSetVehiclePose(pose, true);
  return true;
}

bool AirsimPlannerBridge::sendPose(Eigen::Affine3f eigen_pose) {

  current_goal_ = eigen_pose;

  // Transform pose to airsim
  Eigen::Affine3f airsim_pose = airsimPoseFromWorld(eigen_pose);

  Eigen::Quaternionf rotation(airsim_pose.linear());

  Pose pose = Pose(msr::airlib::Vector3r(airsim_pose.translation().x(),
                                         airsim_pose.translation().y(),
                                         airsim_pose.translation().z()),
                   msr::airlib::Quaternionr(
                       rotation.w(), rotation.x(), rotation.y(), rotation.z()));
  client.simSetVehiclePose(pose, true);
  return true;
}

/*
The following functions are redundant with
the poses transformer but that one is currently
only python.
*/
AirsimPlannerBridge::Pose AirsimPlannerBridge::worldPoseFromAirsim(AirsimPlannerBridge::Pose pos) {
  pos.position.x() = pos.position.x();
  pos.position.y() = -(pos.position.y());
  pos.position.z() = -(pos.position.z());
  pos.orientation.w() = pos.orientation.w();
  pos.orientation.x() = pos.orientation.x();
  pos.orientation.y() = -pos.orientation.y();
  pos.orientation.z() = -pos.orientation.z();

  return pos;
}

Eigen::Affine3f AirsimPlannerBridge::worldPoseFromAirsim(Eigen::Affine3f pos) {
  Eigen::Affine3f new_pos;

  Eigen::Vector3f translation = Eigen::Vector3f(
      pos.translation().x(), -pos.translation().y(), -pos.translation().z());
  new_pos.translation() = translation;

  Eigen::Quaternionf rotation(pos.linear());
  Eigen::Quaternionf new_rotation(rotation.w(), rotation.x(),
                         -rotation.y(), -rotation.z());
  new_pos = new_pos.rotate(new_rotation);

  return new_pos;
}

AirsimPlannerBridge::Pose AirsimPlannerBridge::airsimPoseFromWorld(AirsimPlannerBridge::Pose pos) {
  pos.position.x() = pos.position.x();
  pos.position.y() = -(pos.position.y());
  pos.position.z() = -(pos.position.z());
  pos.orientation.w() = pos.orientation.w();
  pos.orientation.x() = pos.orientation.x();
  pos.orientation.y() = -pos.orientation.y();
  pos.orientation.z() = -pos.orientation.z();
  return pos;
}

Eigen::Affine3f AirsimPlannerBridge::airsimPoseFromWorld(Eigen::Affine3f pos) {
  
  Eigen::Affine3f new_pos(Eigen::Affine3f::Identity());

  Eigen::Vector3f translation = Eigen::Vector3f(
      pos.translation().x(), -pos.translation().y(), -pos.translation().z());
  new_pos.translate(translation);

  Eigen::Quaternionf rotation(pos.linear());
  Eigen::Quaternionf new_rotation(rotation.w(), rotation.x(),
                         -rotation.y(), -rotation.z());
  new_pos = new_pos.rotate(new_rotation);

  return new_pos;
}

bool AirsimPlannerBridge::goalReached(Eigen::Affine3f current_pose) {
  float dist = (current_goal_.translation() - current_pose.translation()).norm();
  float angle = (current_goal_.linear() * current_pose.linear().inverse()).trace();
  bool reached = dist < dist_threshold_ && angle > angle_threshold_;
  if (!reached) {
    // Send pose again
    sendPose(current_goal_);
  }
  return reached;
}

bool AirsimPlannerBridge::isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) {
  // Transform pose to airsim
  Eigen::Affine3f airsim_pose = airsimPoseFromWorld(goal_pose);

  Eigen::Quaternionf rotation(airsim_pose.linear());

  Pose pose = Pose(msr::airlib::Vector3r(airsim_pose.translation().x(),
                                         airsim_pose.translation().y(),
                                         airsim_pose.translation().z()),
                   msr::airlib::Quaternionr(
                       rotation.w(), rotation.x(), rotation.y(), rotation.z()));

  return client.simIsPointOccupied(pose.position, 2.0);
}

BridgeStatus AirsimPlannerBridge::getStatus() {
  return BridgeStatus::IDLE;
}

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools