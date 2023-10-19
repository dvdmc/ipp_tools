/***********************************************************
 *
 * @file: limits.h
 * @breif: Contains simple structures to define limits in different configuration spaces
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef COMMON_LIMITS_H
#define COMMON_LIMITS_H

#include <Eigen/Dense>

namespace ipp_tools
{
namespace common
{

/**
 * @brief Limits in 2D configuration spaces
*/
struct Limits2D {
    Limits2D(Eigen::Vector3d min, Eigen::Vector3d max) : min(min), max(max) {}
    Eigen::Vector3d min;
    Eigen::Vector3d max;
};

/**
 * @brief Limits in 3D configuration spaces
*/
struct Limits3D {
    Limits3D(Eigen::Matrix<double, 6, 1> min, Eigen::Matrix<double, 6, 1> max) : min(min), max(max) {}
    Eigen::Matrix<double, 6, 1> min;
    Eigen::Matrix<double, 6, 1> max;
};

} // namespace common
} // namespace ipp_tools

#endif  // COMMON_LIMITS_H