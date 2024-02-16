#include "ipp_tools/planners/basics/poses_file_planner.h"

namespace ipp_tools {
namespace planners {
namespace basics {

PosesFilePlanner::PosesFilePlanner() : BasePlanner() {}


bool PosesFilePlanner::setup(const std::string& file_path, int subsample) {
  subsample_ = subsample;
  poses_file_ = file_path;

  return true;
}

bool PosesFilePlanner::plan() {
  std::ifstream file(poses_file_);
  std::string str;
  this->path_.clear();  // Clear path

  int i = 0;
  while (std::getline(file, str)) {
    std::istringstream iss(str);
    std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
                                     std::istream_iterator<std::string>());
    // Convert string to float
    std::vector<float> floats;
    std::transform(results.begin(), results.end(), std::back_inserter(floats),
                   [](const std::string& val) { return std::stof(val); });
    // Add pose to path vector with subsample rate
    if (i % subsample_ == 0) {
      Eigen::Translation3f translation(floats[0], floats[1], floats[2]);
      Eigen::Quaternionf rotation(floats[6], floats[3], floats[4], floats[5]);
      Eigen::Affine3f pose = translation * rotation;
      this->path_.push_back(pose);
    }
    i++;
  }
  file.close();
  return true;
}

bool PosesFilePlanner::update() {return true;}

std::vector<Eigen::Affine3f> PosesFilePlanner::getPath() 
{
    if (this->path_.empty()) std::cout << "Path is empty" << std::endl;
    return this->path_;
}

}  // namespace basics
}  // namespace planners
}  // namespace ipp_tools