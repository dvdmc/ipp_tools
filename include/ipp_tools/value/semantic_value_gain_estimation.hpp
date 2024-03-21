/***********************************************************
 *
 * @file: semantic_value_gain_estimation.hpp
 * @breif: Object used to evaluate the information gain of a
 *        viewpoint with respect to the semantic information
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef VALUE_SEMANTIC_VALUE_GAIN_ESTIMATION_HPP
#define VALUE_SEMANTIC_VALUE_GAIN_ESTIMATION_HPP

#include <memory>

#include <Eigen/Dense>

#include <semantic_mapping/VoxelHashMap.hpp>
#include <semantic_mapping/VoxelIntegrator.hpp>

#include <ipp_tools/common/cameras.h>

namespace ipp_tools {
namespace value {

class SemanticEstimationModel {
    /**
     * @brief Base class for semantic estimation models
     */
   public:
    SemanticEstimationModel() = default;

    virtual ~SemanticEstimationModel() = default;

    virtual Eigen::VectorXf getEstimatedClassProbabilities(
        const Eigen::Vector3f &voxel_position, const Eigen::Affine3f &camera,
        semantic_mapping::VoxelInfo *voxel_info) = 0;
};

class SemanticDistanceModel : public SemanticEstimationModel {
    /**
     * @brief Semantic estimation model based on the distance to the camera
     */
   public:
    SemanticDistanceModel(float _max_distance, float _ideal_distance,
                          float _max_probability, float _decay_rate)
        : max_distance(_max_distance),
          ideal_distance(_ideal_distance),
          max_probability(_max_probability),
          decay_rate(_decay_rate) {}

    virtual ~SemanticDistanceModel() = default;

    float max_distance;
    float ideal_distance;
    float max_probability;
    float decay_rate;

    Eigen::VectorXf getEstimatedClassProbabilities(
        const Eigen::Vector3f &voxel_position, const Eigen::Affine3f &camera,
        semantic_mapping::VoxelInfo *voxel_info) {
        float distance = (camera.translation() - voxel_position).norm();

        // The probability of the current class is a parabole with maximum at
        // ideal_distance with max_probability and 1/n_classes at max_distance
        float prob_top;
        if (distance > max_distance) {
            prob_top = 1 / voxel_info->getNumClasses();
        } else {
            prob_top = -std::pow((distance - ideal_distance) * decay_rate, 2) +
                       max_probability;
            prob_top = std::max(0.0f, std::min(1.0f, prob_top));
        }

        int current_class = voxel_info->getMostProbableClass();
        if (current_class < 0) {
            // Classes have all the same probability, return uniform
            return Eigen::ArrayXf::Ones(voxel_info->getNumClasses()) /
                   voxel_info->getNumClasses();
        }
        float prob_rest = (1.0 - prob_top) / (voxel_info->getNumClasses() - 1);
        Eigen::ArrayXf class_ps =
            Eigen::ArrayXf::Ones(voxel_info->getNumClasses()) * prob_rest;
        class_ps(current_class) = prob_top;

        return class_ps;
    }
};

class SemanticValueGainEstimation {
   public:
    /**
     * @brief Construct a new Semantic Value Gain Estimation object
     * @param voxel_hash_map Voxel hash map
     */
    SemanticValueGainEstimation(
        std::shared_ptr<semantic_mapping::VoxelHashMap> _voxel_hash_map,
        std::shared_ptr<semantic_mapping::VoxelIntegrator> _voxel_integrator);

    /**
     * @brief Destroy the Semantic Value Gain Estimation object
     */
    ~SemanticValueGainEstimation() = default;

    /**
     * @brief Evaluate the information gain of a viewpoint with respect to the
     * semantic information
     * @param camera Camera pose
     * @param voxel_position Voxel position
     * @return Information gain
     */
    float evaluateInformationGain(const Eigen::Affine3f &camera,
                                  const Eigen::Vector3f &voxel_position,
                                  SemanticEstimationModel *model, bool skip_other_class=true) const;

   private:
    std::shared_ptr<semantic_mapping::VoxelHashMap> voxel_hash_map_;
    std::shared_ptr<semantic_mapping::VoxelIntegrator> voxel_integrator_;
};

}  // namespace value
}  // namespace ipp_tools

#endif  // VALUE_SEMANTIC_VALUE_GAIN_ESTIMATION_HPP
