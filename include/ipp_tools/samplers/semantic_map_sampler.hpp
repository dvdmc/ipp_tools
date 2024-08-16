/***********************************************************
 *
 * @file: actions_space_sampler.h
 * @breif: Functions to sample action spaces uniformly or randomly.
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef SAMPLERS_SEMANTIC_MAP_SAMPLER_H
#define SAMPLERS_SEMANTIC_MAP_SAMPLER_H

#include <random>

#include <Eigen/Dense>

#include <core_tools/actions.h>
#include <core_tools/angles.h>
#include <core_tools/limits.h>
#include <core_tools/quantiles.h>
#include <core_tools/random.h>

#include <semantic_mapping/VoxelHashMap.hpp>

namespace ipp_tools {
namespace samplers {

/**
 * @brief Sample equispaced actions from the action space
 *        The number of sampled actions will be n^N
 * @param n Number of positions to sample
 * @param current_pose Current pose to compute the action
 * @return A vector of actions
 */
template <int N>
std::vector<Eigen::Matrix<float, N, 1>> sample_semantic_map_random(
    int n, const Eigen::Affine3f& current_pose,
    const semantic_mapping::VoxelHashMap& semantic_map,
    const std::vector<Eigen::Vector3f>& frontier_voxels) {
        
    std::vector<Eigen::Matrix<float, N, 1>> result;

    // From the map, get all the entropies
    std::vector<float> entropies;
    for (auto voxel : semantic_map.getVoxelHashMapData()) {
        entropies.push_back(voxel.second.computeEntropy());
    }
    // Compute the 0.8 quantile
    float quantile = core_tools::quantiles<float>(entropies, {0.2f})[0];

    std::vector<Eigen::Vector3f> uncertain_semantic_voxels;
    for (auto voxel : semantic_map.getVoxelHashMapData()) {
        if (voxel.second.computeEntropy() < quantile) {
            uncertain_semantic_voxels.push_back(
                semantic_map.keyToPoint(voxel.first));
        }
    }

    // Combine frontiers and voxels
    std::vector<Eigen::Vector3f> all_voxels = uncertain_semantic_voxels;
    for (auto frontier_voxel : frontier_voxels) {
        all_voxels.push_back(frontier_voxel);
    }

    // Pick n random voxel positions
    std::vector<Eigen::Vector3f> sampled_voxels;
    std::sample(all_voxels.begin(), all_voxels.end(),
                std::back_inserter(sampled_voxels), n,
                core_tools::get_global_random_generator());

    // For the random positions, compute positions around that look to the
    // center of the voxel Current fix values: distances at 1.0, 1.5, and 2.0. 8
    // orientations around.
    // TODO: Current fix values are based on the action space conf file and
    // action space sampling. Change to parameters Then the action is the
    // difference between the current pose and the computed viewpoint
    for (auto voxel : sampled_voxels) {
        for (float distance : {0.5f, 1.0f, 1.5f}) {
            for (float theta = -M_PI; theta < M_PI; theta += M_PI / 8.0f) {
                Eigen::Matrix<float, N, 1> action;
                Eigen::Vector3f origin =
                    current_pose.inverse() * voxel.cast<float>();
                Eigen::Vector3f translation =
                    current_pose.inverse() *
                    (voxel.cast<float>() +
                     Eigen::Vector3f(distance * std::cos(theta),
                                     distance * std::sin(theta), 0.0f));
                Eigen::Vector3f direction = translation - origin;
                float angle_to_origin =
                    std::atan2(direction.y(), direction.x());
                Eigen::Quaternionf q(current_pose.rotation());
                float delta_theta = core_tools::normalizeAnglePi(
                    M_PI + angle_to_origin -
                    q.toRotationMatrix().eulerAngles(0, 1, 2)[0]);

                action(0, 0) = translation.x();
                action(1, 0) = translation.y();
                action(2, 0) = translation.z();
                action(3, 0) = delta_theta;
                result.push_back(action);
            }
        }
    }

    return result;
}

}  // namespace samplers
}  // namespace ipp_tools

#endif  // SAMPLERS_SEMANTIC_MAP_SAMPLER_H