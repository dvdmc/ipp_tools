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
    ipp_tools::common::Limits<3> limits(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(10, 10, 0));
    // Initialize the map
    ipp_tools::maps::TestMap map(resolution);
    std::shared_ptr<ipp_tools::maps::TestMap> map_ptr = std::make_shared<ipp_tools::maps::TestMap>(map);
    // Initialize the RRT planner
    ipp_tools::planners::RRT<Eigen::Affine2f, Eigen::Vector2f, ipp_tools::common::Limits<3>> rrt(map_ptr, limits);

    // Set the start and goal points
    Eigen::Affine2f start(Eigen::Translation2f(-4.5, -4.5));
    Eigen::Affine2f goal(Eigen::Translation2f(4.5, 4.5));

    // Plan the route
    std::cout << "Creating plan" << std::endl;
    rrt.setup(start, goal);

    std::vector<Eigen::Affine2f> route = rrt.getPath();
    // Transform to points
    std::vector<Eigen::Vector2f> route_points;
    for (int i = 0; i < route.size(); i++)
    {
        route_points.push_back(route[i].translation());
    }
    // Save the route in a file
    std::ofstream file;
    file.open("route.txt");
    for (int i = 0; i < route.size(); i++)
    {
        file << route_points[i].x() << " " << route_points[i].y() << std::endl;
    }
    file.close();

    map_ptr->printRoute(route_points);
    return 0;
}
