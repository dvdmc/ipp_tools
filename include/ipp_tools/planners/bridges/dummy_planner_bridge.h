/***********************************************************
 *
 * @file: dummy_planner_bridge.h
 * @breif: Bridge to not send any pose. Intended for manual control
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PLANNERS_DUMMY_PLANNER_BRIDGE_H
#define PLANNERS_DUMMY_PLANNER_BRIDGE_H

#include <iomanip>
#include <iostream>
#include <Eigen/Dense>

#include <ipp_tools/planners/bridges/base_planner_bridge.h>

namespace ipp_tools {
namespace planners {
namespace bridges {

class DummyPlannerBridge : public BasePlannerBridge {
 public:
  DummyPlannerBridge(/* args */);
  ~DummyPlannerBridge();

  bool sendPose(Eigen::Affine3f eigen_pose) override;

  bool goalReached(Eigen::Affine3f eigen_pose) override;
  bool isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) override;
  BridgeStatus getStatus() override;
  
};

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools


#endif  // PLANNERS_DUMMY_PLANNER_BRIDGE_H