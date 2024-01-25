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

namespace ipp_tools {
namespace planners {
namespace bridges {

class AirsimPlannerBridge {
 private:
  typedef common_utils::Utils Utils;
  typedef msr::airlib::VectorMath VectorMath;
  typedef msr::airlib::Pose Pose;

 public:
  AirsimPlannerBridge(/* args */);
  ~AirsimPlannerBridge();

  bool sendPose(Pose pose);
  bool sendPose(Eigen::Affine3d eigen_pose);

  Pose worldPoseFromAirsim(Pose pos);
  Eigen::Affine3d worldPoseFromAirsim(Eigen::Affine3d pos);

  Pose airsimPoseFromWorld(Pose pos);
  Eigen::Affine3d airsimPoseFromWorld(Eigen::Affine3d pos);

 private:
  msr::airlib::MultirotorRpcLibClient client;
};

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools


#endif  // PLANNERS_AIRSIM_PLANNER_BRIDGE_H