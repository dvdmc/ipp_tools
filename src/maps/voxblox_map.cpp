/**
 * @file   voxblox_map.cpp
 * @author Lukas Schmid, ASL
 *         Modified by David Morilla Cabello, V4RL
 * @date   19.06.2020
 */

#include "ipp_tools/maps/voxblox_map.h"

#include <voxblox/io/sdf_ply.h>

namespace ipp_tools {
namespace maps {

VoxbloxMap::VoxbloxMap(voxblox::EsdfServer *esdf_server)
    : TSDFMap(), esdf_server_(esdf_server) {
    // cache constants
    c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
    c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();
    setCollisionRadius(0.2);
}

bool VoxbloxMap::isTraversable(const Eigen::Vector3f &position) {
    double distance = 0.0;
    if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(
            position.cast<double>(), &distance)) {
        // This means the voxel is observed
        return (distance > p_collision_radius_);
    } else {
        // std::cout << "POINT WAS NOT OBSERVED" << std::endl;
    }
    return false;
}

bool VoxbloxMap::isPathTraversable(const Eigen::Vector3f &start,
                                   const Eigen::Vector3f &end) {
    // Line between start and end
    Eigen::Vector3f direction = end - start;
    float distance = direction.norm();
    direction.normalize();
    float step = c_voxel_size_ / 2.0;
    int steps = std::ceil(distance / step);
    for (int i = 0; i < steps; i++) {
        Eigen::Vector3f point = start + direction * (i * step);
        if (!isTraversable(point)) {
            return false;
        }
    }
    return true;
}

bool VoxbloxMap::isObserved(const Eigen::Vector3f &point) {
    return esdf_server_->getEsdfMapPtr()->isObserved(point.cast<double>());
}

bool VoxbloxMap::exists(const Eigen::Vector3f &point) {
    return isObserved(point);
}

unsigned char VoxbloxMap::getVoxelState(const Eigen::Vector3f &point) {
    double distance = 0.0;
    if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(
            point.cast<double>(), &distance)) {
        // This means the voxel is observed
        if (distance < c_voxel_size_) {
            if (distance < -c_voxel_size_) {
                return VoxbloxMap::INVALID;
            } else {
                return VoxbloxMap::OCCUPIED;
            }
        } else {
            return VoxbloxMap::FREE;
        }
    } else {
        return VoxbloxMap::UNKNOWN;
    }
}

unsigned char VoxbloxMap::getVoxelState(const voxblox::TsdfVoxel &voxel) {
    if (voxel.weight < 1e-6) {
        return VoxbloxMap::UNKNOWN;
    } else if (voxel.distance <= c_voxel_size_/1.2) {
        if (voxel.distance <= -c_voxel_size_/1.2) {
            return VoxbloxMap::INVALID;
        } else {
            return VoxbloxMap::OCCUPIED;
        }
    } else {
        return VoxbloxMap::FREE;
    }
}

float VoxbloxMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool VoxbloxMap::getVoxelCenter(const Eigen::Vector3f &point,
                                Eigen::Vector3f &voxel_center) {
    voxblox::BlockIndex block_id =
        esdf_server_->getEsdfMapPtr()
            ->getEsdfLayerPtr()
            ->computeBlockIndexFromCoordinates(
                point.cast<voxblox::FloatingPoint>());
    voxel_center =
        voxblox::getOriginPointFromGridIndex(block_id, c_block_size_);
    voxblox::VoxelIndex voxel_id =
        voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
            (point - voxel_center).cast<voxblox::FloatingPoint>(),
            1.0 / c_voxel_size_);
    voxel_center +=
        voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_);
    return true;
}

// get the stored TSDF distance
float VoxbloxMap::getVoxelDistance(const Eigen::Vector3f &point) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
        esdf_server_->getTsdfMapPtr()
            ->getTsdfLayerPtr()
            ->getBlockPtrByCoordinates(voxblox_point);
    if (block) {
        voxblox::TsdfVoxel *tsdf_voxel =
            block->getVoxelPtrByCoordinates(voxblox_point);
        if (tsdf_voxel) {
            return tsdf_voxel->distance;
        }
    }
    return 0.0;
}

// get the stored weight
float VoxbloxMap::getVoxelWeight(const Eigen::Vector3f &point) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
        esdf_server_->getTsdfMapPtr()
            ->getTsdfLayerPtr()
            ->getBlockPtrByCoordinates(voxblox_point);
    if (block) {
        voxblox::TsdfVoxel *tsdf_voxel =
            block->getVoxelPtrByCoordinates(voxblox_point);
        if (tsdf_voxel) {
            return tsdf_voxel->weight;
        }
    }
    return 0.0;
}

// get the maximum allowed weight (return 0 if using uncapped weights)
float VoxbloxMap::getMaximumWeight() { return c_maximum_weight_; }

void VoxbloxMap::getTsdfVoxelsBoundingBox(
    const core_tools::BoundingVolume &bounding_box,
    voxblox::HierarchicalIndexMap *block_voxel_list) {
    CHECK_NOTNULL(block_voxel_list);

    voxblox::Layer<voxblox::TsdfVoxel> *layer =
        esdf_server_->getTsdfMapPtr()->getTsdfLayerPtr();
    float voxel_size = layer->voxel_size();
    float voxel_size_inv = 1.0 / layer->voxel_size();
    int voxels_per_side = layer->voxels_per_side();

    const voxblox::FloatingPoint delta_x_in_voxels =
        (bounding_box.max[0] - bounding_box.min[0]) / (2.0 * voxel_size);
    const voxblox::FloatingPoint delta_y_in_voxels =
        (bounding_box.max[1] - bounding_box.min[1]) / (2.0 * voxel_size);
    const voxblox::FloatingPoint delta_z_in_voxels =
        (bounding_box.max[2] - bounding_box.max[2]) / (2.0 * voxel_size);

    const voxblox::Point center(
        (bounding_box.max[0] - bounding_box.min[0]) / 2.0,
        (bounding_box.max[1] - bounding_box.min[1]) / 2.0,
        (bounding_box.max[2] - bounding_box.max[2]) / 2.0);
    const voxblox::GlobalIndex center_index =
        voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(center,
                                                             voxel_size_inv);

    for (voxblox::FloatingPoint x = -delta_x_in_voxels; x <= delta_x_in_voxels;
         x++) {
        for (voxblox::FloatingPoint y = -delta_y_in_voxels;
             y <= delta_y_in_voxels; y++) {
            for (voxblox::FloatingPoint z = -delta_z_in_voxels;
                 z <= delta_z_in_voxels; z++) {
                voxblox::Point point_voxel_space(x, y, z);

                // check if point is inside the volume of interest
                if (point_voxel_space.x() <= delta_x_in_voxels &&
                    point_voxel_space.y() <= delta_y_in_voxels &&
                    point_voxel_space.z() <= delta_z_in_voxels) {
                    voxblox::GlobalIndex voxel_offset_index(
                        std::floor(point_voxel_space.x()),
                        std::floor(point_voxel_space.y()),
                        std::floor(point_voxel_space.z()));
                    // Get the block and voxel indices from this.
                    voxblox::BlockIndex block_index;
                    voxblox::VoxelIndex voxel_index;

                    voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
                        voxel_offset_index + center_index, voxels_per_side,
                        &block_index, &voxel_index);
                    (*block_voxel_list)[block_index].push_back(voxel_index);
                }
            }
        }
    }
}

void VoxbloxMap::getOccupiedPositionsBoundingBox(
    const core_tools::BoundingVolume &bounding_box,
    std::vector<Eigen::Vector3f> &occupied_position) {
    const voxblox::FloatingPoint delta_x =
        bounding_box.max[0] - bounding_box.min[0];
    const voxblox::FloatingPoint delta_y =
        bounding_box.max[1] - bounding_box.min[1];
    const voxblox::FloatingPoint delta_z =
        bounding_box.max[2] - bounding_box.min[2];

    const voxblox::Point center(delta_x / 2.0, delta_y / 2.0, delta_z / 2.0);

    for (voxblox::FloatingPoint x = -delta_x; x <= delta_x;
         x += c_voxel_size_) {
        for (voxblox::FloatingPoint y = -delta_y; y <= delta_y;
             y += c_voxel_size_) {
            for (voxblox::FloatingPoint z = -delta_z; z <= delta_z;
                 z += c_voxel_size_) {
                Eigen::Vector3f point(center.x() + x, center.y() + y,
                                      center.z() + z);
                if (getVoxelState(point) == VoxbloxMap::OCCUPIED) {
                    occupied_position.push_back(point);
                }
            }
        }
    }
}

Eigen::Vector3f VoxbloxMap::getNormalAtPosition(
    const Eigen::Vector3f &voxel_position) {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double dist = 0.0;
    if (getESDFServer().getEsdfMapPtr()->getDistanceAndGradientAtPosition(
            voxel_position.cast<double>(), &dist, &normal)) {
        return normal.normalized().cast<float>();
    } else {
        return Eigen::Vector3f::Zero();
    }
}

std::vector<Eigen::Vector3f> VoxbloxMap::getFrontierVoxelsPositions() {
    std::vector<Eigen::Vector3f> frontier_voxel_positions;
    for (auto frontier_index : frontier_voxels) {
        Eigen::Vector3f position;
        positionFromGlobalIndex(frontier_index, position);
        frontier_voxel_positions.push_back(position);
    }
    return frontier_voxel_positions;
}

voxblox::Mesh VoxbloxMap::getMeshFromTsdf() {
    voxblox::Mesh mesh;
    voxblox::io::convertLayerToMesh<voxblox::TsdfVoxel>(
        *esdf_server_->getTsdfMapPtr()->getTsdfLayerPtr(), &mesh);
    return mesh;
}

voxblox::Mesh VoxbloxMap::getMeshFromEsdf() {
    voxblox::Mesh mesh;
    voxblox::io::convertLayerToMesh<voxblox::EsdfVoxel>(
        *esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr(), &mesh);
    return mesh;
}
}  // end namespace maps
}  // end namespace ipp_tools