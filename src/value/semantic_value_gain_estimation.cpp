#include "ipp_tools/value/semantic_value_gain_estimation.hpp"

namespace ipp_tools {
namespace value {

SemanticValueGainEstimation::SemanticValueGainEstimation(
    std::shared_ptr<semantic_mapping::VoxelHashMap> _voxel_hash_map,
    std::shared_ptr<semantic_mapping::VoxelIntegrator> _voxel_integrator)
    : voxel_hash_map_(_voxel_hash_map), voxel_integrator_(_voxel_integrator) {}

float SemanticValueGainEstimation::evaluateInformationGain(
    const Eigen::Affine3f &camera, const Eigen::Vector3f &voxel_position,
    SemanticEstimationModel *model, bool skip_other_class) const {
    // Get the voxel info
    semantic_mapping::VoxelInfo *voxel_info =
        voxel_hash_map_->getVoxelPtr(voxel_position);
    if (voxel_info == nullptr || (skip_other_class && voxel_info->getMostProbableClass() == 0)) {
        return 0.0;
    }
    float prev_entropy = voxel_info->computeEntropy();

    // Generate estimated semantic information
    Eigen::VectorXf class_ps = model->getEstimatedClassProbabilities(
        voxel_position, camera, voxel_info);

    // Uncertainties are assumed to be 0.00001f (Initial in VoxelInfo is 0.001f by default)
    Eigen::MatrixXf uncertainties =
        Eigen::MatrixXf::Ones(voxel_info->getNumClasses(), 1) * 0.00001f;

    // Samples count is one and the ground truth class count is 0 as it won't be
    // used
    int samples_count = 1;
    Eigen::ArrayXi gt_class_count =
        Eigen::ArrayXi::Zero(voxel_info->getNumClasses());

    std::unique_ptr<semantic_mapping::VoxelInfo> predicted_voxel_info(
        new semantic_mapping::VoxelInfo(class_ps, uncertainties, samples_count,
                                        gt_class_count));

    // Simulate the voxel integration
    semantic_mapping::VoxelInfo integrated_voxel_info =
        voxel_integrator_->fuseVoxel(*voxel_info, *predicted_voxel_info);

    // Get the new entropy
    float new_entropy = integrated_voxel_info.computeEntropy();

    // Compute the information gain (entropy reduction)
    float information_gain = -(prev_entropy - new_entropy);

    return information_gain;
}

}  // namespace value
}  // namespace ipp_tools