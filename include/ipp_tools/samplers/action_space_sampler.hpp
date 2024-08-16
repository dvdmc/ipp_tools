/***********************************************************
 *
 * @file: actions_space_sampler.h
 * @breif: Functions to sample action spaces uniformly or randomly.
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef SAMPLERS_ACTION_SPACE_SAMPLER_H
#define SAMPLERS_ACTION_SPACE_SAMPLER_H

#include <random>

#include <Eigen/Dense>

#include <core_tools/actions.h>
#include <core_tools/limits.h>

namespace ipp_tools {
namespace samplers {

/**
 * @brief Sample equispaced actions from the action space
 *        The number of sampled actions will be n^N
 * @param n Number of samples per dimension
 * @return A vector of actions
 */
template <int N>
std::vector<Eigen::Matrix<float, N, 1>> sample_action_space_uniform(
    int n, const core_tools::ActionSpace<N>& action_space) {
    std::vector<Eigen::Matrix<float, N, 1>> result;

    // Calculate the total number of samples
    int totalSamples = std::pow(n, N);

    for (int i = 0; i < totalSamples; ++i) {
        Eigen::Matrix<float, N, 1> sample;
        int index = i;

        for (int j = 0; j < N; ++j) {
            // Calculate equispaced value for each dimension
            float step = (action_space.max(j) - action_space.min(j)) / (n - 1);
            sample(j) = action_space.min(j) + (index % n) * step;

            // Move to the next dimension
            index /= n;
        }
        result.push_back(sample);
    }

    return result;
}

/**
     * @brief Sample equispaced actions from the action space
     *          specifying the number of samples per dimension
     *          the number of sampled actions will be the product
     *         of the number of samples per dimension
     *
     * @param v_n Number of samples per dimension
     * @return A vector of actions
     */
    template <int N>
    std::vector<Eigen::Matrix<float, N, 1>> sample_action_space_uniform(
        const std::vector<int>& v_n, const core_tools::ActionSpace<N>& action_space) {
        std::vector<Eigen::Matrix<float, N, 1>> result;

        // Calculate the total number of samples
        int totalSamples = 1;
        for (int count : v_n) {
            totalSamples *= count;
        }

        for (int i = 0; i < totalSamples; ++i) {
            Eigen::Matrix<float, N, 1> sample;
            int index = i;

            for (int j = 0; j < N; ++j) {
                int count = v_n[j];
                if (count == 1) {
                    sample(j) = (action_space.max(j) + action_space.min(j)) / 2;
                    continue;
                } else {
                    float step = (action_space.max(j) - action_space.min(j)) / (count - 1);
                    sample(j) = action_space.min(j) + float(index % count) * step;
                }

                index /= count;
            }
            result.push_back(sample);
        }

        return result;
    }


/**
 * @brief Sample random actions from the action space TODO: Homogenize with
 * random pose generator
 * @param n Number of samples
 * @return A vector of actions
 */
template <int N>
std::vector<Eigen::Matrix<float, N, 1>> sample_action_space_random(
    int n, const core_tools::ActionSpace<N>& action_space) {
    std::vector<Eigen::Matrix<float, N, 1>> actions;
    Eigen::Matrix<float, N, 1> action;
    for (int i = 0; i < n; i++) {
        action = action_space.min +
                 Eigen::Matrix<float, N, 1>::Random().cwiseAbs().cwiseProduct(
                     action_space.max - action_space.min);
        actions.push_back(action);
    }
    return actions;
}

}  // namespace samplers
}  // namespace ipp_tools

#endif  // SAMPLERS_ACTION_SPACE_SAMPLER_H