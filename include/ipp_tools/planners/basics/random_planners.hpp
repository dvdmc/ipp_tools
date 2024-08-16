/***********************************************************
 *
 * @file: random_planners.h
 * @breif: Planner that obtains poses from a file currently
 *       only supports 3D poses in the format:
 *      x y z qw qx qy qz
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PLANNERS_RANDOM_PLANNERS_H
#define PLANNERS_RANDOM_PLANNERS_H

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <ipp_tools/maps/map.h>
#include <core_tools/actions.h>
#include <core_tools/limits.h>
#include <ipp_tools/samplers/random_pose_generator.hpp>
#include <ipp_tools/planners/base_planner.h>

namespace ipp_tools {
namespace planners {
namespace basics {

class Random3dPlanner : public BasePlanner<Eigen::Affine3f> {
   public:
    /**
     * @brief Construct a new Random 3d Planner object
     */
    Random3dPlanner();

    /**
     * @brief Destroy the Poses File Planner object
     */
    ~Random3dPlanner() = default;

    /**
     * @brief Sets up the parameters for the planner
     * 
     */
    bool setup(std::shared_ptr<ipp_tools::maps::Map<Eigen::Vector3f>> map,
               int num_steps,
               core_tools::ActionSpace<4> action_space,
               core_tools::Polygonal3DVolume map_volume);

    /**
     * @brief Plan using the configuration in the param map
     */
    bool plan() override;

    /**
     * @brief Update the planner
     */
    bool update() override;

    /**
     * @brief Get the path
     * @return std::vector<Eigen::Affine3f> Path
     */
    std::vector<Eigen::Affine3f> getPath() override;

    /**
     * @brief Update the planner data
     * @param current_pose
     * @return if the update was successful
     */
    bool updateData(const Eigen::Affine3f& current_pose);

   private:
    std::shared_ptr<ipp_tools::maps::Map<Eigen::Vector3f>> map_;
    int num_steps_;
    std::unique_ptr<core_tools::ActionSpace<4>> yaw_action_space_;
    std::unique_ptr<core_tools::ActionSpace<6>> action_space_;
    std::unique_ptr<core_tools::Polygonal3DVolume> map_volume_;
    std::unique_ptr<ipp_tools::samplers::RandomPoseGenerator<Eigen::Affine3f>> pose_generator_;
    std::vector<Eigen::Affine3f> path_;

    bool is_setup_;
    bool is_data_updated_;
    Eigen::Affine3f current_pose_;
};

}  // namespace basics
}  // namespace planners
}  // namespace ipp_tools

#endif  // PLANNERS_RANDOM_PLANNERS_H