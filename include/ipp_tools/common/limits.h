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
 * @brief Limit in N-dimensional configuration spaces
*/
template <int N>
struct Limits {
    Limits(Eigen::Matrix<float, N, 1> _min, Eigen::Matrix<float, N, 1> _max) : min(_min), max(_max) {}
    Eigen::Matrix<float, N, 1> min;
    Eigen::Matrix<float, N, 1> max;
};

/**
 * @brief Bounding volume in 3D space TODO: Homogenize with the limits
*/
class BoundingVolume : public Limits<3>{
 public:
  BoundingVolume() : Limits<3>(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()) {}
  BoundingVolume(const Eigen::Vector3f &min, const Eigen::Vector3f &max) : Limits<3>(min, max) {}

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
};

class BoundingVolumeIterator {
 public:
  BoundingVolumeIterator(const BoundingVolume &volume, const float &step)
      : volume_(volume), step_(step) {
    current_ = volume_.min;
  }

  Eigen::Vector3f next() {
    Eigen::Vector3f next = current_;
    current_.x() += step_;
    if (current_.x() > volume_.max.x()) {
      current_.x() = volume_.min.x();
      current_.y() += step_;
      if (current_.y() > volume_.max.y()) {
        current_.y() = volume_.min.y();
        current_.z() += step_;
      }
    }
    return next;
  }

  bool hasNext() {
    return current_.z() <= volume_.max.z();
  }

  private:
    BoundingVolume volume_;
    float step_;
    Eigen::Vector3f current_;
};

class Polygonal3DVolume {
 public:
  Polygonal3DVolume(std::vector<Eigen::Vector2f> _vertices, Eigen::Vector2f _height_limits)
      : vertices(_vertices), height_limits(_height_limits) {
        // Generate planes. Each two consecutive vertices define a plane. Compute the normal
        // assuming vertical walls (polygon is in the XY plane) and normal pointing inwards
        for (int i = 0; i < vertices.size(); i++) {
          Eigen::Vector2f v1 = vertices[i];
          Eigen::Vector2f v2 = vertices[(i + 1) % vertices.size()];
          Eigen::Vector2f edge = v2 - v1;
          Eigen::Vector2f middle_point = (v1 + v2) / 2;
          float z_mid = (height_limits.x() + height_limits.y()) / 2;
          Eigen::Vector3f center(middle_point.x(), middle_point.y(), z_mid);
          Eigen::Vector2f normal2d = -Eigen::Vector2f(-edge.y(), edge.x()).normalized(); // (-) Inwards
          Eigen::Vector3f normal(normal2d.x(), normal2d.y(), 0);
          normals.push_back(normal);
          centers.push_back(center);
          half_sizes.push_back(edge.norm() / 2);
        }
      }
  ~Polygonal3DVolume() {}
  
  bool contains(const Eigen::Vector3f &point) const {
    // Point in plane frame
    if (point.z() < height_limits.x() || point.z() > height_limits.y())
    {
      return false;
    }

    for (int i = 0; i < normals.size(); i++) {
      // We first check if the point is within the region of interest of a wall
      // Move the point to the center of the wall coord
      Eigen::Vector2f v_p_c = Eigen::Vector2f(point.x(), point.y()) - Eigen::Vector2f(centers[i].x(), centers[i].y());
      // Project the point on the normal
      float proj_c_n = v_p_c.dot(normals[i].head<2>());
      // Get the distance from the point to the projected point
      float dist_p_n = (v_p_c - proj_c_n * normals[i].head<2>()).norm();
      if (dist_p_n < half_sizes[i])
      {
        if ((centers[i]-point).dot(normals[i]) > 0)
        {
          return false;
        }
      }
    }
    return true;
  }

  BoundingVolume getBoundingVolume() const {
    Eigen::Vector3f min = Eigen::Vector3f(vertices[0].x(), vertices[0].y(), height_limits.x());
    Eigen::Vector3f max = Eigen::Vector3f(vertices[0].x(), vertices[0].y(), height_limits.y());
    for (int i = 1; i < vertices.size(); i++) {
      min.x() = std::min(min.x(), vertices[i].x());
      min.y() = std::min(min.y(), vertices[i].y());
      max.x() = std::max(max.x(), vertices[i].x());
      max.y() = std::max(max.y(), vertices[i].y());
    }
    return BoundingVolume(min, max);
  }

  Eigen::Vector3f getCenter() const {
    // TODO: In the future, this should be the avg of the
    // 3 or 4 closest centers to avoid weird shapes affecting
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (int i = 0; i < centers.size(); i++) {
      center += centers[i];
    }
    return center / centers.size();
  }

  // Overload << operator to print the polygonal volume
  friend std::ostream& operator<<(std::ostream& os, const Polygonal3DVolume& volume) {
    os << "Polygonal3DVolume: " << std::endl;
    os << "Vertices: " << std::endl;
    for (int i = 0; i < volume.vertices.size(); i++) {
      os << volume.vertices[i].transpose() << std::endl;
    }
    os << "Normals: " << std::endl;
    for (int i = 0; i < volume.normals.size(); i++) {
      os << volume.normals[i].transpose() << std::endl;
    }
    os << "Centers: " << std::endl;
    for (int i = 0; i < volume.centers.size(); i++) {
      os << volume.centers[i].transpose() << std::endl;
    }
    os << "Height limits: " << volume.height_limits.transpose() << std::endl;
    return os;
  }

  std::vector<Eigen::Vector2f> vertices;
  std::vector<Eigen::Vector3f> normals, centers;
  std::vector<float> half_sizes;
  Eigen::Vector2f height_limits;
};

class Polygonal3DVolumeIterator {
 /*
  This makes use of the BoundingVolume class to iterate over the volume
  and just checks if the point is inside the polygonal volume
 */
 public:
  Polygonal3DVolumeIterator(const Polygonal3DVolume &volume, const float &step)
      : volume_(volume), step_(step), bounding_volume_iterator_(volume.getBoundingVolume(), step) {
    current_ = bounding_volume_iterator_.next();
  }

  Eigen::Vector3f next() {
    Eigen::Vector3f next = current_;
    current_ = bounding_volume_iterator_.next();
    while (!volume_.contains(current_)) {
      if (!bounding_volume_iterator_.hasNext()) {
        break;
      }
      current_ = bounding_volume_iterator_.next();
    }
    return next;
  }

  bool hasNext() {
    return bounding_volume_iterator_.hasNext();
  }

  private:
    Polygonal3DVolume volume_;
    float step_;
    BoundingVolumeIterator bounding_volume_iterator_;
    Eigen::Vector3f current_;
};

} // namespace common
} // namespace ipp_tools

#endif  // COMMON_LIMITS_H