#include <iostream>

#include "ipp_tools/planners/active_perception_planner.h"

namespace ipp_tools {
namespace planners {

ActivePerceptionPlanner::ActivePerceptionPlanner() : BasePlanner() {}

bool ActivePerceptionPlanner::setup(
    const float& point_size, const common::CameraData& camera_data,
    const common::ActionSpace<4>& action_space) {
    point_size_ = point_size;
    camera_data_ = std::make_shared<common::CameraData>(camera_data);
    action_space_ = std::make_shared<common::ActionSpace<4>>(action_space);

    viewpoint_evaluator_ =
        std::make_shared<value::ViewpointEvaluator>(camera_data_);

    return true;
}

bool ActivePerceptionPlanner::plan() {
    this->path_.clear();  // Clear path
    update();             // Update once to get the first viewpoint
    return;
}

bool ActivePerceptionPlanner::update() {
    // Sample uniformly from the action space
    std::vector<Eigen::Matrix<double, 4, 1>> actions =
        action_space_->sample(3);  // 3 per axis

    // Evaluate the viewpoints
    std::vector<Eigen::Affine3f> viewpoint_candidates(actions.size());
    std::vector<float> v_viewpoint_value(actions.size());

    for (int i = 0; i < actions.size(); i++) {
        Eigen::Affine3f viewpoint_candidate = current_pose_;
        viewpoint_candidate.translation() +=
            Eigen::Vector3f(actions[i].block<3, 1>(0, 0));
        viewpoint_candidate.rotate(
            Eigen::AngleAxisf(actions[i](3, 0), Eigen::Vector3d::UnitZ()));
        // TODO: Check limits and traversability
        viewpoint_candidates[i] = viewpoint_candidate;
        v_viewpoint_value[i] =
            viewpoint_evaluator_->evaluateViewpointVisibleFrontiers(
                viewpoint_candidate, point_size_, frontier_voxels_,
                surface_voxels_);
    }

    // Select the best viewpoint
    int best_viewpoint_idx =
        std::distance(v_viewpoint_value.begin(),
                      std::max_element(v_viewpoint_value.begin(),
                                       v_viewpoint_value.end()));

    // Add the best viewpoint to the path 
    // TODO: Reorder if value is higher than the ones that are already stored
    // Note: Regardless, this planner is supposed to be greedy so one goal at a time.
    Eigen::Affine3d best_viewpoint_d =
        viewpoint_candidates[best_viewpoint_idx].cast<double>();
    this->path_.push_back(best_viewpoint_d);

    return true;
}

bool ActivePerceptionPlanner::updateData(
    const std::vector<Eigen::Vector3f>& frontier_voxels,
    const std::vector<Eigen::Vector3f>& surface_voxels,
    const Eigen::Affine3f& current_pose) {
    frontier_voxels_ = frontier_voxels;
    surface_voxels_ = surface_voxels;
    current_pose_ = current_pose;

    return true;
}

std::vector<Eigen::Affine3d> ActivePerceptionPlanner::getPath() {
    if (this->path_.empty()) std::cout << "Path is empty" << std::endl;
    return this->path_;
}

}  // namespace planners
}  // namespace ipp_tools