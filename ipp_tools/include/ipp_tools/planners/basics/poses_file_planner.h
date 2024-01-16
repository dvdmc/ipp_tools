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

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <ipp_tools/planners/base_planner.h>

namespace ipp_tools
{
namespace planners
{
namespace basics
{

class PosesFilePlanner : public BasePlanner<Eigen::Affine3d>
{
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
  void plan() override;

  /**
   * @brief Update the planner
   */
  void update() override;

private:
  int subsample_;
  std::string poses_file_;
};

bool PosesFilePlanner::setup(const std::string& file_path, int subsample)
{
  subsample_ = subsample;
  poses_file_ = file_path;
  
  return true;
}

void PosesFilePlanner::plan()
{
  std::ifstream file(poses_file_); 
  std::string str;
  path_.clear(); // Clear path

  int i = 0;
  while (std::getline(file, str))
  {
    std::istringstream iss(str);
    std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
                                    std::istream_iterator<std::string>());
    // Convert string to float
    std::vector<float> floats;
    std::transform(results.begin(), results.end(), std::back_inserter(floats), [](const std::string& val) {
        return std::stof(val);
    });
    // Add pose to path vector with subsample rate
    if (i % subsample_ == 0) {
        Eigen::Translation3d translation(floats[0], floats[1], floats[2]);
        Eigen::Quaterniond rotation(floats[6],floats[3], floats[4], floats[5]);
        Eigen::Affine3d pose = translation * rotation;
        path_.push_back(pose);
    }
    i++;
  }
  file.close();
}

}  // namespace basics
}  // namespace planners
}  // namespace ipp_tools

#endif  // PLANNERS_POSES_FILE_PLANNER_H