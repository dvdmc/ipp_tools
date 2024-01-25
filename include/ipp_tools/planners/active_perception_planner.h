/***********************************************************
 *
 * @file: active_perception_planner.h
 * @breif: Planner that uses semantic information to plan 
 *         the next best view
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PLANNERS_ACTIVE_PERCEPTION_PLANNER_H
#define PLANNERS_ACTIVE_PERCEPTION_PLANNER_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>

#include <ipp_tools/common/actions.h>
#include <ipp_tools/common/cameras.h>

#include <ipp_tools/value/viewpoint_evaluator.hpp>
#include <ipp_tools/planners/base_planner.h>

namespace ipp_tools {
namespace planners {

class ActivePerceptionPlanner : public BasePlanner<Eigen::Affine3d> {
 public:
    /**
     * @brief Construct a new Active Perception Planner object
     */ 
  ActivePerceptionPlanner();
  
    /**
     * @brief Destroy the Active Perception Planner object
     */
  ~ActivePerceptionPlanner();

    /**
     * @brief Sets up the parameters for the planner
     * @param TODO
     * @return True if the setup is successful
     */
    bool setup(const float &point_size, 
               const common::CameraData& camera_data, 
               const common::ActionSpace<4>& action_space);

    /**
     * @brief Plan using the configuration in the param map
     */
    bool plan() override;

    /**
     * @brief Update the planner
     */
    bool update() override;

    /**
     * @brief Provide updated information to the planner to
     *        compute new viewpoint values and update the
     *        planner
     */
    bool updateData(const std::vector<Eigen::Vector3f>& frontier_voxels,
                     const std::vector<Eigen::Vector3f>& surface_voxels,
                     const Eigen::Affine3f& current_pose);

    /**
     * @brief Get the path
     * @return std::vector<Eigen::Affine3d> Path
     */
    std::vector<Eigen::Affine3d> getPath() override;

    private:
    
    Eigen::Affine3f current_pose_; // Current pose of the camera
    float point_size_; // Point size used to estimate occlusions.

    std::vector<Eigen::Vector3f> frontier_voxels_;
    std::vector<Eigen::Vector3f> surface_voxels_;

    std::shared_ptr<common::CameraData> camera_data_;
    std::shared_ptr<common::ActionSpace<4>> action_space_;
    std::shared_ptr<value::ViewpointEvaluator> viewpoint_evaluator_;
};  // class ActivePerceptionPlanner

}  // namespace planners
}  // namespace ipp_tools

#endif  // PLANNERS_ACTIVE_PERCEPTION_PLANNER_H