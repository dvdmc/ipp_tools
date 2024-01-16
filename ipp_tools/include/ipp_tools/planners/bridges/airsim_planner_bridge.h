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

class AirsimPlannerBridge {
 private:
  typedef common_utils::Utils Utils;
  typedef msr::airlib::VectorMath VectorMath;
  typedef msr::airlib::Pose Pose;

 public:
  AirsimPlannerBridge(/* args */);
  ~AirsimPlannerBridge();

  bool sendPose(Eigen::Affine3d eigen_pose) {
    Pose pose = Pose(msr::airlib::Vector3r(eigen_pose.translation().x(),
                                           eigen_pose.translation().y(),
                                           eigen_pose.translation().z()),
                     msr::airlib::Quaternionr(
                         eigen_pose.rotation().w(), eigen_pose.rotation().x(),
                         eigen_pose.rotation().y(), eigen_pose.rotation().z()));
    Pose airsim_pose = airsimPoseFromWorld_(pose);
    client.simSetVehiclePose(airsim_pose, true);
  }

 private:
  msr::airlib::MultirotorRpcLibClient client;

  /*
  The following functions are redundant with
  the poses transformer but that one is currently
  only python.
  */

  Pose worldPoseFromAirsim_(Pose pos) {
    pos.position.x() = pos.position.x();
    pos.position.y() = -(pos.position.y());
    pos.position.z() = -(pos.position.z());
    pos.orientation.w() = pos.orientation.w();
    pos.orientation.x() = pos.orientation.x();
    pos.orientation.y() = -pos.orientation.y();
    pos.orientation.z() = -pos.orientation.z();

    return pos;
  }

  Pose airsimPoseFromWorld_(Pose pos) {
    pos.position.x() = pos.position.x();
    pos.position.y() = -(pos.position.y());
    pos.position.z() = -(pos.position.z());
    pos.orientation.w() = pos.orientation.w();
    pos.orientation.x() = pos.orientation.x();
    pos.orientation.y() = -pos.orientation.y();
    pos.orientation.z() = -pos.orientation.z();

    return pos;
  }
};

AirsimPlannerBridge::AirsimPlannerBridge(/* args */) {
  client.confirmConnection();

  // Get pose from Airsim
  const auto& pose = client.simGetVehiclePose()
}

AirsimPlannerBridge::~AirsimPlannerBridge() {}

}  // namespace planners
}  // namespace ipp_tools

#endif  // PLANNERS_AIRSIM_PLANNER_BRIDGE_H