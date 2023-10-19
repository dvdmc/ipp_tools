#include <maps/test_map.h>

namespace ipp_tools
{
namespace maps
{

TestMap::TestMap(double resolution)
{
    // Set the resolution
    resolution_ = resolution;

    // Set the grid size
    grid_size_ = map_size_/resolution_;

    // Set the circle radius
    circle_radius_ = circle_size_/resolution_;

    // Initialize the map
    map_data_.resize(grid_size_);
    for (int i = 0; i < grid_size_; i++)
    {
        map_data_[i].resize(grid_size_);
    }

    // Fill the map
    for (int i = 0; i < grid_size_; i++)
    {
        for (int j = 0; j < grid_size_; j++)
        {
            // Check if the point is inside the circle
            if (pow(i - map_center_, 2) + pow(j - map_center_, 2) <= pow(circle_radius_, 2))
            {
                map_data_[i][j] = 0;
            }
            else
            {
                map_data_[i][j] = 255;
            }
        }
    }
}

bool TestMap::isTraversable(const Eigen::Affine2d& point)
{
    // Get the map point
    Eigen::Vector2i map_point;
    worldToMap_(point, map_point);

    // Check if the point is inside the map
    if (map_point.x() < 0 || map_point.x() >= grid_size_ || map_point.y() < 0 || map_point.y() >= grid_size_)
    {
        return false;
    }

    // Check if the point is inside the circle
    if (pow(map_point.x() - map_center_, 2) + pow(map_point.y() - map_center_, 2) <= pow(circle_radius_, 2))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool TestMap::exists(const Eigen::Affine2d& point)
{
    // Get the map point
    Eigen::Vector2i map_point;
    worldToMap_(point, map_point);

    // Check if the point is inside the map
    if (map_point.x() < 0 || map_point.x() >= grid_size_ || map_point.y() < 0 || map_point.y() >= grid_size_)
    {
        return false;
    }

    // Check if the point is inside the circle
    if (pow(map_point.x() - map_center_, 2) + pow(map_point.y() - map_center_, 2) <= pow(circle_radius_, 2))
    {
        return true;
    }
    else
    {
        return false;
    }
}

unsigned char TestMap::getVoxelState(const Eigen::Affine2d& point)
{
    // Get the map point
    Eigen::Vector2i map_point;
    worldToMap_(point, map_point);

    // Check if the point is inside the map
    if (map_point.x() < 0 || map_point.x() >= grid_size_ || map_point.y() < 0 || map_point.y() >= grid_size_)
    {
        return 255;
    }

    // Return the voxel state
    return map_data_[map_point.x()][map_point.y()];
}

bool TestMap::getVoxelCenter(const Eigen::Affine2d& point, Eigen::Affine2d& voxel_center)
{
    // Get the map point
    Eigen::Vector2i map_point;
    worldToMap_(point, map_point);

    // Check if the point is inside the map
    if (map_point.x() < 0 || map_point.x() >= grid_size_ || map_point.y() < 0 || map_point.y() >= grid_size_)
    {
        return false;
    }

    // Get the voxel center
    voxel_center.translation().x() = map_point.x()*resolution_ - map_size_/2 + resolution_/2;
    voxel_center.translation().y() = map_point.y()*resolution_ - map_size_/2 + resolution_/2;

    return true;

}

void TestMap::worldToMap_(const Eigen::Affine2d& point, Eigen::Vector2i& map_point)
{
    // Get the map point
    map_point.x() = (point.translation().x() + map_size_/2)/resolution_;
    map_point.y() = (point.translation().y() + map_size_/2)/resolution_;

    // Check if the point is inside the map
    if (map_point.x() < 0 || map_point.x() >= grid_size_ || map_point.y() < 0 || map_point.y() >= grid_size_)
    {
        map_point.x() = -1;
        map_point.y() = -1;
    }

}



} // namespace maps
} // namespace ipp_tools
