#include "ipp_tools/value/semantic_value_gain_estimation.hpp"

namespace ipp_tools {
namespace value {

SemanticValueGainEstimation::SemanticValueGainEstimation(
    std::shared_ptr<semantic_mapping::VoxelHashMap> _voxel_hash_map)
    : voxel_hash_map_(_voxel_hash_map) {}

float SemanticValueGainEstimation::evaluateInformationGain(
    const Eigen::Affine3f &camera, const Eigen::Vector3f &voxel_position,
    SemanticEstimationModel* model) const {
        
    // Get the voxel info
    semantic_mapping::VoxelInfo *voxel_info =
        voxel_hash_map_->getVoxelPtr(voxel_position);
    if (voxel_info == nullptr) {
        return 0.0;
    }
    float prev_entropy = voxel_info->computeEntropy();

    // Generate estimated semantic information
    Eigen::VectorXf class_ps = model->getEstimatedClassProbabilities(
        voxel_position, camera, *voxel_info);

    // Uncertainties are assumed to be 0.1 for all classes
    Eigen::MatrixXf uncertainties =
        Eigen::MatrixXf::Ones(voxel_info->getNumClasses(), 1) * 0.1;

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
        voxel_hash_map_->getIntegratorPtr()->fuseVoxel(*voxel_info,
                                                       *predicted_voxel_info);

    // Get the new entropy
    float new_entropy = integrated_voxel_info.computeEntropy();

    // Compute the information gain
    float information_gain = prev_entropy - new_entropy;

    return information_gain;
}

}  // namespace value
}  // namespace ipp_tools