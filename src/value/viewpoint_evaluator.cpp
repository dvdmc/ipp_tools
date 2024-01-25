#include <ipp_tools/value/viewpoint_evaluator.hpp>

namespace ipp_tools {
namespace value {

ViewpointEvaluator::ViewpointEvaluator(common::CameraData _camera_data)
    : camera_data_(_camera_data) {}

// TODO: All of this can be done in vectorized form

bool ViewpointEvaluator::isPointVisible(const Eigen::Vector3f &point,
                                        const Eigen::Affine3f &camera) {
    // Transform point to camera frame
    Eigen::Vector3f point_camera = camera.inverse() * point;

    // Check if point is in front of the camera and inside the frustum
    camera_data_.isInsideFrustum(point_camera);

    return true;
}

void ViewpointEvaluator::visbilityCulling(
    const std::vector<Eigen::Vector3f> &points, const Eigen::Affine3f &camera,
    std::vector<bool> &v_is_point_visible) {
    for (int i = 0; i < points.size(); i++) {
        v_is_point_visible[i] = isPointVisible(points[i], camera);
    }
}

void ViewpointEvaluator::occlusionCulling(
    const std::vector<Eigen::Vector3f> &points, const float &point_size,
    const Eigen::Affine3f &camera, std::vector<bool> &v_is_point_visible) {
    // Transform points to camera frame
    std::vector<std::pair<int, Eigen::Vector3f>> points_in_camera(
        points.size());
    for (int i = 0; i < points.size(); i++) {
        Eigen::Vector3f point_camera = camera.inverse() * points[i];
        points_in_camera[i] = std::make_pair(i, point_camera);
    }

    // Order points by depth and maintain index (Hi-Z buffer)
    std::sort(points_in_camera.begin(), points_in_camera.end(),
              [](const std::pair<int, Eigen::Vector3f> &a,
                 const std::pair<int, Eigen::Vector3f> &b) {
                  return a.second(0) < b.second(0);
              });

    // Check occlusion
    // We loop over all points in camera from near to far
    // The remaining points are tested for occlussion.
    // If they are occluded, they are removed from the list
    // and thus not iterated
    for (int i = 0; i < points_in_camera.size(); i++) {
        Eigen::Vector3f point_i = points_in_camera[i].second;
        // Add it to visible points
        v_is_point_visible[points_in_camera[i].first] = true;

        float shadow_radius = point_size * 0.5f;

        // Vector to store points to remove
        std::vector<int> to_remove;
        for (int j = i + 1; j < points_in_camera.size(); j++) {
            Eigen::Vector3f point_j = points_in_camera[j].second;

            // Project point to point_i plane considering perspective
            // and depth of point_i and point_j
            float depth_i = point_i(0);
            float depth_j = point_j(0);
            float x = point_j(1) * depth_i / depth_j;
            float y = point_j(2) * depth_i / depth_j;
            point_j = Eigen::Vector3f(depth_i, x, y);
            // Check if point is inside the shadow radius
            if ((point_i - point_j).norm() < shadow_radius) {
                // Point is occluded
                to_remove.push_back(j);
            }
        }
        // Remove points from points_camera list
        for (int j = to_remove.size() - 1; j >= 0; j--) {
            points_in_camera.erase(points_in_camera.begin() + to_remove[j]);
        }
    }
}

void ViewpointEvaluator::visiblityAndOcclusionCulling(
    const std::vector<Eigen::Vector3f> &points, const float &point_size,
    const Eigen::Affine3f &camera, std::vector<bool> &v_is_point_visible) {
    // Check visibility
    visbilityCulling(points, camera, v_is_point_visible);

    std::vector<Eigen::Vector3f> points_reduced;
    std::vector<int> points_reduced_idx;
    for (int i = 0; i < points.size(); i++) {
        if (v_is_point_visible[i]) {
            points_reduced.push_back(points[i]);
            points_reduced_idx.push_back(i);
        }
    }
    std::vector<bool> v_is_point_visible_reduced(points_reduced.size(), false);

    // Check occlusion
    occlusionCulling(points_reduced, point_size, camera,
                     v_is_point_visible_reduced);

    // Upate visible points vector
    for (int i = 0; i < points_reduced_idx.size(); i++) {
        v_is_point_visible[points_reduced_idx[i]] =
            v_is_point_visible_reduced[i];
    }
}

float ViewpointEvaluator::evaluateViewpointVisibleFrontiers(
    const Eigen::Affine3f &camera, const float &point_size,
    const std::vector<Eigen::Vector3f> &frontier_voxels,
    const std::vector<Eigen::Vector3f> &surface_voxels) {
    // Evaluate which voxels are visible from the camera. Surface and frontier
    // voxels are evaluated together
    std::vector<bool> v_is_point_visible(
        frontier_voxels.size() + surface_voxels.size(), false);
    std::vector<Eigen::Vector3f> all_voxels_positions;
    all_voxels_positions.insert(all_voxels_positions.end(),
                                frontier_voxels.begin(), frontier_voxels.end());
    all_voxels_positions.insert(all_voxels_positions.end(),
                                surface_voxels.begin(), surface_voxels.end());
    visiblityAndOcclusionCulling(all_voxels_positions, point_size, camera,
                                 v_is_point_visible);

    // Compute the value as the number of visible frontier voxels
    int visible_frontier_voxels = 0;
    for (int i = 0; i < frontier_voxels.size(); i++) {
        if (v_is_point_visible[i]) {
            visible_frontier_voxels++;
        }
    }
    return visible_frontier_voxels;
}

}  // namespace value
}  // namespace ipp_tools