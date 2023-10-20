/***********************************************************
 *
 * @file: random_pose_generator.h
 * @breif: Contains random pose samplers for different configuration spaces
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#ifndef RANDOM_POSE_GENERATOR_H
#define RANDOM_POSE_GENERATOR_H

#include <random>

#include <Eigen/Dense>
 
#include "common/limits.h"

// Two templated classes for Affine2D and Affine3D

namespace ipp_tools
{
namespace samplers
{
/**
 * @brief Random pose generator for 2D configuration spaces
*/
template <typename Xn>
class RandomPoseGenerator;

/**
 * @brief Random pose generator for 2D configuration spaces
*/
template <>
class RandomPoseGenerator<Eigen::Affine2d>
{
public:
    RandomPoseGenerator(const common::Limits2D &limits, unsigned int seed = 0)
        : generator_(seed),
            x_distribution_(limits.min(0), limits.max(0)),
            y_distribution_(limits.min(1), limits.max(1)),
            yaw_distribution_(limits.min(2), limits.max(2)) {}

    ~RandomPoseGenerator() = default;

    void getPose(Eigen::Affine2d& pose)
    {
        pose.translation().x() = x_distribution_(generator_);
        pose.translation().y() = y_distribution_(generator_);
        pose.linear() = Eigen::Rotation2Dd(yaw_distribution_(generator_)).toRotationMatrix();
    }

private:
    std::mt19937 generator_;
    std::uniform_real_distribution<double> x_distribution_;
    std::uniform_real_distribution<double> y_distribution_;
    std::uniform_real_distribution<double> yaw_distribution_;
};

/**
 * @brief Random pose generator for 3D configuration spaces
*/
template <>
class RandomPoseGenerator<Eigen::Affine3d>
{
public:
    RandomPoseGenerator(const common::Limits3D &limits, unsigned int seed = 0)
        : generator_(seed),
            x_distribution_(limits.min(0), limits.max(0)),
            y_distribution_(limits.min(1), limits.max(1)),
            z_distribution_(limits.min(2), limits.max(2)),
            roll_distribution_(limits.min(3), limits.max(3)),
            pitch_distribution_(limits.min(4), limits.max(4)),
            yaw_distribution_(limits.min(5), limits.max(5)) {}

    ~RandomPoseGenerator() = default;

    void getPose(Eigen::Affine3d& pose)
    {
        pose.translation().x() = x_distribution_(generator_);
        pose.translation().y() = y_distribution_(generator_);
        pose.translation().z() = z_distribution_(generator_);
        pose.linear() = (Eigen::AngleAxisd(roll_distribution_(generator_), Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pitch_distribution_(generator_), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw_distribution_(generator_), Eigen::Vector3d::UnitZ()))
                            .toRotationMatrix();
    }

private:
    std::mt19937 generator_;
    std::uniform_real_distribution<double> x_distribution_;
    std::uniform_real_distribution<double> y_distribution_;
    std::uniform_real_distribution<double> z_distribution_;
    std::uniform_real_distribution<double> roll_distribution_;
    std::uniform_real_distribution<double> pitch_distribution_;
    std::uniform_real_distribution<double> yaw_distribution_;
};

} // namespace samplers
} // namespace ipp_tools

#endif  // RANDOM_POSE_GENERATOR_H