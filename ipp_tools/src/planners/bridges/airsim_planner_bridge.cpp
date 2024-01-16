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

bool AirsimPlannerBridge::sendPose(Eigen::Affine3d eigen_pose) {

  Eigen::Quaterniond rotation(eigen_pose.linear());

  Pose pose = Pose(msr::airlib::Vector3r(eigen_pose.translation().x(),
                                         eigen_pose.translation().y(),
                                         eigen_pose.translation().z()),
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

Eigen::Affine3d AirsimPlannerBridge::worldPoseFromAirsim(Eigen::Affine3d pos) {
  Eigen::Affine3d new_pos;

  Eigen::Vector3d translation = Eigen::Vector3d(
      pos.translation().x(), -pos.translation().y(), -pos.translation().z());
  new_pos.translation() = translation;

  Eigen::Quaterniond rotation(pos.rotation());
  Eigen::Quaterniond new_rotation(rotation.w(), rotation.x(),
                         -rotation.y(), -rotation.z());
  new_pos.rotate(new_rotation);

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

Eigen::Affine3d AirsimPlannerBridge::airsimPoseFromWorld(Eigen::Affine3d pos) {
  
  Eigen::Affine3d new_pos;

  Eigen::Vector3d translation = Eigen::Vector3d(
      pos.translation().x(), -pos.translation().y(), -pos.translation().z());
  new_pos.translation() = translation;

  Eigen::Quaterniond rotation(pos.rotation());
  Eigen::Quaterniond new_rotation(rotation.w(), rotation.x(),
                         -rotation.y(), -rotation.z());
  new_pos.rotate(new_rotation);

  return new_pos;
}

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools