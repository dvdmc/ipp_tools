#include <fstream>
#include <iostream>

#include "ipp_tools/planners/rrt.h"
#include "ipp_tools/common/limits.h"
#include "ipp_tools/maps/test_map.h"

// This code is used to test the RRT algorithm
// It generates an instance fo the test_map, which is a hardcoded map.
// Then it initializes the RRT planner and tries to plan a fixed route.
// The route is then saved in a file
int main(int argc, char** argv)
{
    std::cout << "Starting test" << std::endl;
    double resolution = 0.1;
    ipp_tools::common::Limits2D limits(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(10, 10, 0));
    // Initialize the map
    ipp_tools::maps::TestMap map(resolution);
    std::shared_ptr<ipp_tools::maps::TestMap> map_ptr = std::make_shared<ipp_tools::maps::TestMap>(map);
    // Initialize the RRT planner
    ipp_tools::planners::RRT<Eigen::Affine2d, ipp_tools::common::Limits2D> rrt(map_ptr, limits);

    // Set the start and goal points
    Eigen::Affine2d start(Eigen::Translation2d(-4.5, -4.5));
    Eigen::Affine2d goal(Eigen::Translation2d(4.5, 4.5));

    // Plan the route
    std::cout << "Creating plan" << std::endl;
    rrt.setup(start, goal);

    std::vector<Eigen::Affine2d> route = rrt.getPath();
    
    // Save the route in a file
    std::ofstream file;
    file.open("route.txt");
    for (int i = 0; i < route.size(); i++)
    {
        file << route[i].translation().x() << " " << route[i].translation().y() << std::endl;
    }
    file.close();

    map_ptr->printRoute(route);
    return 0;
}
