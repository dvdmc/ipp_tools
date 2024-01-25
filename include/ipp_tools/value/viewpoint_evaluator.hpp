/***********************************************************
 *
 * @file: viewpoint_evaluator.hpp
 * @breif: Object used to evaluate the quality of a viewpoint
 *         with common metrics such as entropy, mutual information
 *         volumetric information gain, etc.
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef VALUE_VIEWPOINT_EVALUATOR_HPP
#define VALUE_VIEWPOINT_EVALUATOR_HPP

#include <Eigen/Dense>

#include <ipp_tools/common/cameras.h>

namespace ipp_tools {
namespace value {

class ViewpointEvaluator {
   public:
    /**
     * @brief Construct a new Viewpoint Evaluator object
     */
    ViewpointEvaluator(common::CameraData _camera_data);

    /**
     * @brief Destroy the Viewpoint Evaluator object
     */
    ~ViewpointEvaluator() = default;

    /**
     * @brief Evaluate the visibility of a point from a camera
     * @param point Point to evaluate
     * @param camera Camera from which the point is evaluated
     * @return True if the point is visible from the camera
     */
    bool isPointVisible(const Eigen::Vector3f &point,
                        const Eigen::Affine3f &camera);

    /**
     * @brief Checks if the point if they are visible in the camera frustrum or
     * not
     *
     * @param points list of points to be evaluated
     * @param camera camera pose
     * @param v_is_point_visible vector of booleans indicating if the point is
     * visible or not
     */
    void visbilityCulling(const std::vector<Eigen::Vector3f> &points,
                          const Eigen::Affine3f &camera,
                          std::vector<bool> &v_is_point_visible);

    /**
     * @brief Checks if the point are inter-occluded or not
     *
     * @param points list of points to be evaluated
     * @param camera camera pose
     * @param v_is_point_visible vector of booleans indicating if the point is
     * visible or not
     */
    void occlusionCulling(const std::vector<Eigen::Vector3f> &points,
                          const float &point_size,
                          const Eigen::Affine3f &camera,
                          std::vector<bool> &v_is_point_visible);

    /**
     * @brief Performs visibility and occlusion culling on the points without
     * repeating operations
     *
     * @param points list of points to be evaluated
     * @param camera camera pose
     * @param v_is_point_visible vector of booleans indicating if the point is
     * visible or not
     */
    void visiblityAndOcclusionCulling(
        const std::vector<Eigen::Vector3f> &points, const float &point_size,
        const Eigen::Affine3f &camera, std::vector<bool> &v_is_point_visible);

    /**
     * @brief Evaluate a viewpoint for volumetric gain checking for visibility
     * and occlusion
     *
     * @param camera  camera pose
     * @param frontier_voxels list of frontier voxels
     * @param surface_voxels list of surface voxels
     * @return float volumetric gain
     */
    float evaluateViewpointVisibleFrontiers(
        const Eigen::Affine3f &camera,
        const float &point_size,
        const std::vector<Eigen::Vector3f> &frontier_voxels,
        const std::vector<Eigen::Vector3f> &surface_voxels);

    // float evaluateViewpointFrontiersAndEntropy(
    //     const Eigen::Affine3f &camera,
    //     const std::vector<Eigen::Vector3f> &frontier_voxels,
    //     const std::vector<Eigen::Vector3f> &surface_voxels,
    //     const std::vector<VoxelInfo *> &voxels);

   private:
    const common::CameraData camera_data_;
};

}  // namespace value
}  // namespace ipp_tools
#endif  // VALUE_VIEWPOINT_EVALUATOR_HPP