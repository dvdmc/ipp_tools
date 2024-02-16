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

#include "ipp_tools/common/limits.h"
#include "ipp_tools/maps/tsdf_map.h"

namespace ipp_tools
{
namespace maps
{

class VoxbloxMap : public TSDFMap<Eigen::Vector3f> {
 public:
  VoxbloxMap(voxblox::EsdfServer *esdf_server);

  ~VoxbloxMap() {}

  // check collision for a single pose
  virtual bool isTraversable(const Eigen::Vector3f &position) override;

  // check collision for a line between two poses
  virtual bool isPathTraversable(const Eigen::Vector3f &start,
                             const Eigen::Vector3f &end) override;

  // check if a point exists in the map
  virtual bool exists(const Eigen::Vector3f &point) override;

  // check whether point is part of the map
  bool isObserved(const Eigen::Vector3f &point);

  // get occupancy // TODO: Check if this could be abstract
  virtual unsigned char getVoxelState(const Eigen::Vector3f &point) override;

  // get voxel size
  virtual float getVoxelSize() override;

  // get the center of a voxel from input point
  virtual bool getVoxelCenter(const Eigen::Vector3f &point,
                              Eigen::Vector3f &voxel_center) override;

  // get the stored distance
  virtual float getVoxelDistance(const Eigen::Vector3f &point) override;

  // get the stored weight
  virtual float getVoxelWeight(const Eigen::Vector3f &point) override;

  // get the maximum allowed weight (return 0 if using uncapped weights)
  virtual float getMaximumWeight() override;

  // Utilities for extracting voxels in a query bounding box
  // TODO: Check this function
  void getTsdfVoxelsBoundingBox(
      const ipp_tools::common::BoundingVolume &bounding_box,
      voxblox::HierarchicalIndexMap *block_voxel_list);

  void getOccupiedPositionsBoundingBox(
      const common::BoundingVolume &bounding_box,
      std::vector<Eigen::Vector3f> &occupied_position);

  Eigen::Vector3f getNormalAtPosition(const Eigen::Vector3f &voxel);

  /**
   * Convert a position in world frame into global index
   * @param[in] position position in world frame
   * @param[out] global_voxel_index global voxel index at position
   * @param[in] allocate allocate map position if it was not already allocated
   * @return true, if given position was already allocated in the map
   */
  inline bool globalIndexFromPosition(const Eigen::Vector3f& position,
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
      Eigen::Vector3f& position) {
    int voxels_per_side = getTsdfLayer()->voxels_per_side();
    voxblox::BlockIndex block_index;
    voxblox::VoxelIndex voxel_index;
    voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
        global_voxel_index, voxels_per_side, &block_index, &voxel_index);

    voxblox::Point pos_f = getTsdfLayer()
                           ->getBlockByIndex(block_index)
                           .computeCoordinatesFromVoxelIndex(voxel_index);
    position = pos_f;
  }


  // set parameters
  void setCollisionRadius(const float collision_radius) {
    p_collision_radius_ = collision_radius;
    esdf_server_->setTraversabilityRadius(p_collision_radius_);
  }

  void setMaxVoxelWeight(const float max_weigth) {
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
  float p_collision_radius_;

  // cache constants
  float c_voxel_size_;
  float c_block_size_;
  float c_maximum_weight_;
};  // end class VoxbloxMap

}  // end namespace maps
}  // end namespace ipp_tools

#endif  // MAPS_VOXBLOX_MAP_H_
