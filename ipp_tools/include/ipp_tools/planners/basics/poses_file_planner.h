/***********************************************************
 *
 * @file: poses_file_planner.h
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
#ifndef PLANNERS_POSES_FILE_PLANNER_H
#define PLANNERS_POSES_FILE_PLANNER_H

#include <ipp_tools/planners/base_planner.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace ipp_tools {
namespace planners {
namespace basics {

class PosesFilePlanner : public BasePlanner<Eigen::Affine3d> {
 public:
  /**
   * @brief Construct a new Poses File Planner object
   * @param file_path Path to the file containing the poses
   */
  PosesFilePlanner();

  /**
   * @brief Destroy the Poses File Planner object
   */
  ~PosesFilePlanner() = default;

  /**
   * @brief Sets up the parameters for the planner
   * @param file_path Path to the file containing the poses
   * @return True if the setup is successful
   */
  bool setup(const std::string& file_path, int subsample = 5);

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
   * @return std::vector<Eigen::Affine3d> Path
   */
  std::vector<Eigen::Affine3d> getPath() override;

 private:
  int subsample_;
  std::string poses_file_;
};

}  // namespace basics
}  // namespace planners
}  // namespace ipp_tools

#endif  // PLANNERS_POSES_FILE_PLANNER_H