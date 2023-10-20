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
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace ipp_tools
{
namespace ros_utils
{
    template <typename Xn>
    geometry_msgs::msg::PoseStamped affineToPoseStamped(const Xn& affine);

    Eigen::Affine2d convertAffine3dToAffine2d(const Eigen::Affine3d& affine3d);

    template <>
    geometry_msgs::msg::PoseStamped affineToPoseStamped(const Eigen::Affine2d& affine)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = affine.translation().x();
        pose.pose.position.y = affine.translation().y();
        pose.pose.position.z = 0.0;

        // Turn the rotation 2D into a 3D matrix
        Eigen::Matrix3d rot;
        rot.setIdentity();
        rot.topLeftCorner<2, 2>() = affine.rotation();
        
        Eigen::Quaterniond q(rot);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        return pose;
    }

    template <>
    geometry_msgs::msg::PoseStamped affineToPoseStamped(const Eigen::Affine3d& affine)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = affine.translation().x();
        pose.pose.position.y = affine.translation().y();
        pose.pose.position.z = affine.translation().z();

        Eigen::Quaterniond q(affine.rotation());
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        return pose;
    }

    Eigen::Affine2d convertAffine3dToAffine2d(const Eigen::Affine3d& affine3d)
    {
        Eigen::Affine2d affine2d;

        // Extract the 2x2 rotation matrix from the 3x3 rotation matrix
        Eigen::Matrix2d rotation2d = affine3d.rotation().topLeftCorner<2, 2>();

        // Set the translation components
        affine2d.translation() = affine3d.translation().head<2>();

        // Set the 2D rotation
        affine2d.linear() = rotation2d;

        return affine2d;
    }
    
} // namespace ros_utils
} // namespace ipp_tools

#endif  // ROS_UTILS_H