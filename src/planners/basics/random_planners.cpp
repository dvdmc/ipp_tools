#include <ipp_tools/planners/basics/random_planners.hpp>

namespace ipp_tools {
namespace planners {
namespace basics {

Random3dPlanner::Random3dPlanner() : BasePlanner<Eigen::Affine3f>(), is_setup_(false), is_data_updated_(false) {}

bool Random3dPlanner::setup(
    std::shared_ptr<ipp_tools::maps::Map<Eigen::Vector3f>> map, int num_steps,
    core_tools::ActionSpace<4> action_space,
    core_tools::Polygonal3DVolume map_volume) {
    is_setup_ = true;
    std::cout << "Setting up random planner" << std::endl;
    std::cout << "Action space: " << action_space.min.transpose() << " "
              << action_space.max.transpose() << std::endl;
    map_ = map;
    num_steps_ = num_steps;
    yaw_action_space_ =
        std::make_unique<core_tools::ActionSpace<4>>(action_space);

    // Create a new action space with 0,0 for pitch and roll
    Eigen::Matrix<float, 6, 1> min;
    Eigen::Matrix<float, 6, 1> max;
    min << action_space.min(0), action_space.min(1), action_space.min(2), 0, 0, action_space.min(3);
    max << action_space.max(0), action_space.max(1), action_space.max(2), 0, 0, action_space.max(3);
    core_tools::Limits<6> limits(min, max);

    // TODO: Check how to cast the action space to limits
    std::vector<std::string> names{"x", "y", "z", "roll", "pitch", "yaw"};
    action_space_ =
        std::make_unique<core_tools::ActionSpace<6>>(names, min, max);

    map_volume_ =
        std::make_unique<core_tools::Polygonal3DVolume>(map_volume);

    pose_generator_ = std::make_unique<
        ipp_tools::samplers::RandomPoseGenerator<Eigen::Affine3f>>(limits);

    return true;
}

bool Random3dPlanner::plan() {
    if (!is_setup_) {
        std::cout << "WARN: Planner not setup" << std::endl;
    }
    if (!is_data_updated_) {
        std::cout << "WARN: Planner data not updated" << std::endl;
    }

    // Generate random path
    path_.clear();
    int num_attempts = 0;
    for (int i = 0; i < num_steps_; i++) {
        Eigen::Affine3f pose;
        pose_generator_->getPoseFromAction(current_pose_, pose);

        while((!map_volume_->contains(pose.translation()) ||
               !map_->isTraversable(pose.translation())) &&
               num_attempts < 10000) {
            num_attempts++;
            pose_generator_->getPoseFromAction(current_pose_, pose);
        }
        if (num_attempts >= 10000) {
            std::cout << "Could not find valid pose" << std::endl;
            return false;
        }
        path_.push_back(pose);
    }

    return true;
}

bool Random3dPlanner::updateData(const Eigen::Affine3f& current_pose) {
    is_data_updated_ = true;
    current_pose_ = current_pose;
    return true;
}

bool Random3dPlanner::update() { return plan(); }

std::vector<Eigen::Affine3f> Random3dPlanner::getPath() { return path_; }

}  // namespace basics
}  // namespace planners
}  // namespace ipp_tools