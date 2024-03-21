/***********************************************************
 *
 * @file: airsim_planner_bridge.h
 * @breif: Bridge to send poses to airsim
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PLANNERS_AIRSIM_PLANNER_BRIDGE_H
#define PLANNERS_AIRSIM_PLANNER_BRIDGE_H

#include <iomanip>
#include <iostream>
#include <Eigen/Dense>

#include "common/ClockFactory.hpp"
#include "common/Common.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/common_utils/ProsumerQueue.hpp"
#include "common/common_utils/RandomGenerator.hpp"
#include "common/common_utils/StrictMode.hpp"
#include "common/common_utils/Utils.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include <ipp_tools/planners/bridges/base_planner_bridge.h>

namespace ipp_tools {
namespace planners {
namespace bridges {

class AirsimPlannerBridge : public BasePlannerBridge {
 private:
  typedef common_utils::Utils Utils;
  typedef msr::airlib::VectorMath VectorMath;
  typedef msr::airlib::Pose Pose;

 public:
  AirsimPlannerBridge(/* args */);
  ~AirsimPlannerBridge();

  bool sendPose(Pose pose);
  bool sendPose(Eigen::Affine3f eigen_pose) override;

  Pose worldPoseFromAirsim(Pose pos);
  Eigen::Affine3f worldPoseFromAirsim(Eigen::Affine3f pos);

  Pose airsimPoseFromWorld(Pose pos);
  Eigen::Affine3f airsimPoseFromWorld(Eigen::Affine3f pos);

  bool goalReached(Eigen::Affine3f eigen_pose) override;
  bool isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) override;
  BridgeStatus getStatus() override;
  
  void setThresholds(float dist_threshold, float angle_threshold) {
    dist_threshold_ = dist_threshold;
    angle_threshold_ = angle_threshold;
  }

 private:
  msr::airlib::MultirotorRpcLibClient client;

  Eigen::Affine3f current_goal_;
  float dist_threshold_ = 0.1;
  float angle_threshold_ = 0.15;
};

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools


#endif  // PLANNERS_AIRSIM_PLANNER_BRIDGE_H