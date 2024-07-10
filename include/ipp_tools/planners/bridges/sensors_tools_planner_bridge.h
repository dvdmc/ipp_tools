/***********************************************************
 *
 * @file: sensors_tools_planner_bridge.h
 * @breif: Bridge to send poses to the move() interface of sensors_tools
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PLANNERS_SENSORS_TOOLS_PLANNER_BRIDGE_H
#define PLANNERS_SENSORS_TOOLS_PLANNER_BRIDGE_H

#include <iomanip>
#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ipp_tools/planners/bridges/base_planner_bridge.h>

namespace ipp_tools {
namespace planners {
namespace bridges {

class SensorsToolsPlannerBridge : public BasePlannerBridge {

 public:
  SensorsToolsPlannerBridge(ros::NodeHandle &nh);
  ~SensorsToolsPlannerBridge();

  bool sendPose(Eigen::Affine3f eigen_pose) override;
  bool sendPose(geometry_msgs::PoseStamped pose);

  bool goalReached(Eigen::Affine3f eigen_pose) override;
  bool isGoalReachible(Eigen::Affine3f current_pose, Eigen::Affine3f goal_pose) override;
  BridgeStatus getStatus() override;
  
  void setThresholds(float dist_threshold, float angle_threshold) {
    dist_threshold_ = dist_threshold;
    angle_threshold_ = angle_threshold;
  }

 private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    // To check if goal is reachible
    ros::ServiceClient plan_srv_;

    Eigen::Affine3f current_goal_;
    float dist_threshold_ = 0.08;
    float angle_threshold_ = 0.05;
};

}  // namespace bridges
}  // namespace planners
}  // namespace ipp_tools


#endif  // PLANNERS_SENSORS_TOOLS_PLANNER_BRIDGE_H