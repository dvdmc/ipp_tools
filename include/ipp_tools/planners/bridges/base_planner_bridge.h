/***********************************************************
 *
 * @file: base_planner_bridge.h
 * @breif: Base bridge implementing the interface
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PLANNERS_BASE_PLANNER_BRIDGE_H
#define PLANNERS_BASE_PLANNER_BRIDGE_H

#include <iomanip>
#include <iostream>

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

namespace ipp_tools {
namespace planners {
namespace bridges {

enum BridgeStatus { IDLE = 0, RUNNING = 1, ERROR = 2 };

class BasePlannerBridge {
 public:

  virtual bool sendPose(Eigen::Affine3f eigen_pose) = 0;

  virtual bool goalReached(Eigen::Affine3f eigen_pose) = 0;
  virtual bool isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) = 0;
  virtual BridgeStatus getStatus() = 0;
};

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools


#endif  // PLANNERS_BASE_PLANNER_BRIDGE_H