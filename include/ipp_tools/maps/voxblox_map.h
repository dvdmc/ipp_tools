/**
 * @file   voxblox_map.h
 * @author Lukas Schmid, ASL
 *         Modified by David Morilla Cabello, Universidad de Zaragoza
 * @date   14.10.2021
 */
#ifndef MAPS_VOXBLOX_MAP_H_
#define MAPS_VOXBLOX_MAP_H_

#include <voxblox/utils/planning_utils.h>
#include <voxblox_ros/esdf_server.h>
#include <memory>

#include "semantic_mapping/map/tsdf_map.h"

namespace ipp_tools
{
namespace maps
{

// TODO: Move to a common class
class BoundingVolume {
 public:
  BoundingVolume() : min(0, 0, 0), max(0, 0, 0) {}
  BoundingVolume(const Eigen::Vector3d &_min, const Eigen::Vector3d &_max)
      : min(_min), max(_max) {}

  inline bool contains(const Eigen::Vector3d &point) const {
    return (point.x() >= min.x() && point.x() <= max.x() &&
            point.y() >= min.y() && point.y() <= max.y() &&
            point.z() >= min.z() && point.z() <= max.z());
  }

  inline bool intersects(const BoundingVolume &other) const {
    return (min.x() <= other.max.x() && max.x() >= other.min.x() &&
            min.y() <= other.max.y() && max.y() >= other.min.y() &&
            min.z() <= other.max.z() && max.z() >= other.min.z());
  }

  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

class VoxbloxMap : public TSDFMap<Eigen::Vector3d> {
 public:
  VoxbloxMap(voxblox::EsdfServer *esdf_server);

  ~VoxbloxMap() {}

  // check collision for a single pose
  virtual bool isTraversable(const Eigen::Vector3d &position) override;

  // check whether point is part of the map
  virtual bool isObserved(const Eigen::Vector3d &point) override;

  // get occupancy
  virtual unsigned char getVoxelState(const Eigen::Vector3d &point) override;

  // get voxel size
  virtual double getVoxelSize() override;

  // get the center of a voxel from input point
  virtual bool getVoxelCenter(Eigen::Vector3d *center,
                              const Eigen::Vector3d &point) override;

  // get the stored distance
  virtual double getVoxelDistance(const Eigen::Vector3d &point) override;

  // get the stored weight
  virtual double getVoxelWeight(const Eigen::Vector3d &point) override;

  // get the maximum allowed weight (return 0 if using uncapped weights)
  virtual double getMaximumWeight() override;

  // Utilities for extracting voxels in a query bounding box
  // TODO: Check this function
  void getTsdfVoxelsBoundingBox(
      const BoundingVolume &bounding_box,
      voxblox::HierarchicalIndexMap *block_voxel_list);

  void getOccupiedPositionsBoundingBox(
      const BoundingVolume &bounding_box,
      std::vector<Eigen::Vector3d> &occupied_position);

  Eigen::Vector3d getNormalAtPosition(const Eigen::Vector3d &voxel);

  /**
   * Convert a position in world frame into global index
   * @param[in] position position in world frame
   * @param[out] global_voxel_index global voxel index at position
   * @param[in] allocate allocate map position if it was not already allocated
   * @return true, if given position was already allocated in the map
   */
  inline bool globalIndexFromPosition(const Eigen::Vector3d& position,
                                      voxblox::GlobalIndex& global_voxel_index,
                                      bool allocate = false) {
    voxblox::Point pos_f(position.cast<float>());
    auto block_ptr = getTsdfLayer()->getBlockPtrByCoordinates(pos_f);

    if (block_ptr == nullptr) {
      if (allocate) {
        block_ptr = getTsdfLayer()->allocateBlockPtrByCoordinates(pos_f);
      } else {
        return false;
      }
    }

    voxblox::BlockIndex block_index = block_ptr->block_index();
    voxblox::VoxelIndex voxel_index =
        block_ptr->computeVoxelIndexFromCoordinates(pos_f);
    int voxels_per_side = block_ptr->voxels_per_side();
    global_voxel_index = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
        block_index, voxel_index, voxels_per_side);

    return true;
  }

  /**
   * Get global position from voxel index
   * @param[in] global_voxel_index input voxel index
   * @param[out] position output position in world frame
   */
  inline void positionFromGlobalIndex(
      const voxblox::GlobalIndex& global_voxel_index,
      Eigen::Vector3d& position) {
    int voxels_per_side = getTsdfLayer()->voxels_per_side();
    voxblox::BlockIndex block_index;
    voxblox::VoxelIndex voxel_index;
    voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side, &block_index, &voxel_index);

    voxblox::Point pos_f = getTsdfLayer()
                           ->getBlockByIndex(block_index)
                           .computeCoordinatesFromVoxelIndex(voxel_index);
    position = pos_f.cast<double>();
  }


  // set parameters
  void setCollisionRadius(const double collision_radius) {
    p_collision_radius_ = collision_radius;
    esdf_server_->setTraversabilityRadius(p_collision_radius_);
  }

  void setMaxVoxelWeight(const double max_weigth) {
    c_maximum_weight_ = max_weigth;
  }

  voxblox::LongIndexSet frontier_voxels;  
  std::vector<Eigen::Vector3f> getFrontierVoxelsPositions();

  // accessor to the server for specialized planners
  inline voxblox::EsdfServer &getESDFServer() { return *esdf_server_; }

  inline voxblox::Layer<voxblox::TsdfVoxel> *getTsdfLayer() {
    return esdf_server_->getTsdfMapPtr()->getTsdfLayerPtr();
  }

  inline voxblox::Layer<voxblox::EsdfVoxel> *getEsdfLayer() {
    return esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr();
  }

  voxblox::Mesh getMeshFromTsdf();

  voxblox::Mesh getMeshFromEsdf();

 protected:
  // esdf server that contains the map, subscribe to external ESDF/TSDF updates
  // This is NOT-OWNED by this class
  voxblox::EsdfServer *esdf_server_;

  // Parameters
  double p_collision_radius_;

  // cache constants
  double c_voxel_size_;
  double c_block_size_;
  double c_maximum_weight_;
};  // end class VoxbloxMap

}  // end namespace maps
}  // end namespace ipp_tools

#endif  // MAPS_VOXBLOX_MAP_H_
