/***********************************************************
 *
 * @file: limits.h
 * @breif: Contains simple structures to define limits in 
 *         different configuration spaces 
 *          TODO: Reimplement through the actions class
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
    Limits2D(Eigen::Vector3f min, Eigen::Vector3f max) : min(min), max(max) {}
    Eigen::Vector3f min;
    Eigen::Vector3f max;
};

/**
 * @brief Limits in 3D configuration spaces
*/
struct Limits3D {
    Limits3D(Eigen::Matrix<float, 6, 1> min, Eigen::Matrix<float, 6, 1> max) : min(min), max(max) {}
    Eigen::Matrix<float, 6, 1> min;
    Eigen::Matrix<float, 6, 1> max;
};

/**
 * @brief Bounding volume in 3D space TODO: Homogenize with the limits
*/
class BoundingVolume {
 public:
  BoundingVolume() : min(0, 0, 0), max(0, 0, 0) {}
  BoundingVolume(const Eigen::Vector3f &_min, const Eigen::Vector3f &_max)
      : min(_min), max(_max) {}

  bool contains(const Eigen::Vector3f &point) const {
    return (point.x() >= min.x() && point.x() <= max.x() &&
            point.y() >= min.y() && point.y() <= max.y() &&
            point.z() >= min.z() && point.z() <= max.z());
  }

  bool intersects(const BoundingVolume &other) const {
    return (min.x() <= other.max.x() && max.x() >= other.min.x() &&
            min.y() <= other.max.y() && max.y() >= other.min.y() &&
            min.z() <= other.max.z() && max.z() >= other.min.z());
  }

  Eigen::Vector3f min;
  Eigen::Vector3f max;
};

} // namespace common
} // namespace ipp_tools

#endif  // COMMON_LIMITS_H