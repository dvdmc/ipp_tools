/***********************************************************
 *
 * @file: actions.h
 * @breif: Contains common action structures and functions
 *          such as to define action spaces, etc.
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef COMMON_ACTIONS_H
#define COMMON_ACTIONS_H

#include <Eigen/Dense>

namespace ipp_tools
{
namespace common
{

/**
 * @brief Action space in ND
 * 
 * @tparam N Dimension of the action space
 * 
 * @param names Names of the action dimensions
 * @param min Minimum values for the action space
 * @param max Maximum values for the action space
*/
template <int N>
struct ActionSpace {
    ActionSpace(std::vector<std::string> names, Eigen::Matrix<double, N, 1> min, Eigen::Matrix<double, N, 1> max) : min(min), max(max) {}
    std::vector<std::string> names;
    Eigen::Matrix<double, N, 1> min;
    Eigen::Matrix<double, N, 1> max;

    /**
     * @brief Sample equispaced actions from the action space
     *        The number of sampled actions will be n^N
     * @param n Number of samples per dimension
     * @return A vector of actions
    */
    std::vector<Eigen::Matrix<double, N, 1>> sample(int n) {
        std::vector<Eigen::Matrix<double, N, 1>> result;

        // Calculate the total number of samples
        int totalSamples = std::pow(n, N);

        for (int i = 0; i < totalSamples; ++i) {
            Eigen::Matrix<double, N, 1> sample;
            int index = i;

            for (int j = 0; j < N; ++j) {
                // Calculate equispaced value for each dimension
                double step = (max(j) - min(j)) / (n - 1);
                sample(j) = min(j) + (index % n) * step;

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
    std::vector<Eigen::Matrix<double, N, 1>> sample(const std::vector<int>& v_n) {
        assert(v_n.size() == N);
        std::vector<Eigen::Matrix<double, N, 1>> result;
        
        // Calculate the total number of samples
        int totalSamples = 1;
        for (int count : v_n) {
            totalSamples *= count;
        }

        for (int i = 0; i < totalSamples; ++i) {
        Eigen::Matrix<double, N, 1> sample;
        int index = i;

        for (int j = 0; j < N; ++j) {
            int count = v_n[j];
            if(count == 1) {
                sample(j) = (max(j) + min(j)) / 2;
                continue;
            } else
            {
            double step = (max(j) - min(j)) / (count - 1);
            sample(j) = min(j) + float(index % count) * step;
            }

            index /= count;
        }
        result.push_back(sample);
    }

    return result;
    }

    /**
     * @brief Sample random actions from the action space
     * @param n Number of samples
     * @return A vector of actions
    */
    std::vector<Eigen::Matrix<double, N, 1>> sample_random(int n) {
        std::vector<Eigen::Matrix<double, N, 1>> actions;
        Eigen::Matrix<double, N, 1> action;
        for (int i = 0; i < n; i++) {
            action = min + Eigen::Matrix<double, N, 1>::Random().cwiseAbs().cwiseProduct(max - min);
            actions.push_back(action);
        }
        return actions;
    }
   
};

} // namespace common
} // namespace ipp_tools

#endif  // COMMON_ACTIONS_H
