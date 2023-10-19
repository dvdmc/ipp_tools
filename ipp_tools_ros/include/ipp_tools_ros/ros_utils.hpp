/***********************************************************
 *
 * @file: ros_utils.h
 * @breif: Contains useful functions related to ROS
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace ipp_tools
{
namespace ros_utils
{
    template <typename Xn>
    geometry_msgs::msg::PoseStamped affineToPoseStamped(const Xn& affine);

} // namespace ros_utils
} // namespace ipp_tools

#endif  // ROS_UTILS_H