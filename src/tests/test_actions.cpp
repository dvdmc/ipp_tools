#include <fstream>
#include <iostream>

#include "ipp_tools/common/actions.h"

// This code is used to test the actions algorithms
// It generates an instance of the actions space and samples
// actions from it
int main(int argc, char** argv)
{
    std::cout << "Starting test" << std::endl;
    // Initialize the action space
    ipp_tools::common::ActionSpace<4> action_space({"x", "y", "z", "yaw"}, Eigen::Vector4f(-1, -1, -1, -1), Eigen::Vector4f(1, 1, 1, 1));
    // Sample actions
    std::vector<Eigen::Matrix<float, 4, 1>> actions = action_space.sample(3); // 4 per axis

    // Print the sampled actions
    for (int i = 0; i < actions.size(); i++)
    {
        std::cout << actions[i].transpose() << std::endl;
    }

    std::cout << "Second test" << std::endl;
    // Sample actions with different number of samples per dimension (3,2,3,1)
    std::vector<int> v_n = {3, 2, 3, 1};
    actions = action_space.sample(v_n);

    // Print the sampled actions
    for (int i = 0; i < actions.size(); i++)
    {
        std::cout << actions[i].transpose() << std::endl;
    }
    
    return 0;
}
