#include "ipp_tools/planners/bridges/dummy_planner_bridge.h"

namespace ipp_tools {
namespace planners {
namespace bridges {

DummyPlannerBridge::DummyPlannerBridge(/* args */) {
}

DummyPlannerBridge::~DummyPlannerBridge() {}

bool DummyPlannerBridge::sendPose(Eigen::Affine3f eigen_pose) {
  return true;
}

bool DummyPlannerBridge::goalReached(Eigen::Affine3f current_pose) {
  return true;
}

bool DummyPlannerBridge::isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) {
  return true;
}

BridgeStatus DummyPlannerBridge::getStatus() {
  return BridgeStatus::IDLE;
}

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools