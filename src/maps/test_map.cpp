#include <ipp_tools/maps/test_map.h>

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

    // Set the map center
    map_center_ = map_size_/2;

    // Set the grid center
    grid_center_ = grid_size_/2;

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
            if (pow(i - grid_center_, 2) + pow(j - grid_center_, 2) <= pow(circle_radius_, 2))
            {
                map_data_[i][j] = 255;
            }
            else
            {
                map_data_[i][j] = 0;
            }
        }
    }
}

bool TestMap::isTraversable(const Eigen::Vector2d& point)
{
    // Get the value for that cell
    unsigned char value = getVoxelState(point);
    // Check if the cell is occupied
    if (value == OCCUPIED)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool TestMap::isPathTraversable(const Eigen::Vector2d& start, const Eigen::Vector2d& goal)
{
    Eigen::Vector2d direction = (goal - start).normalized();
    Eigen::Vector2d point = start;
    while ((point - goal).norm() > resolution_)
    {
        point += direction*resolution_;
        if (!isTraversable(point))
        {
            return false;
        }
    }
    return true;
}

bool TestMap::exists(const Eigen::Vector2d& point)
{
    // Get the value for that cell
    unsigned char value = getVoxelState(point);
    // Check if the cell is known
    if (value == UNKNOWN)
    {
        return false;
    }
    else
    {
        return true;
    }
}

unsigned char TestMap::getVoxelState(const Eigen::Vector2d& point)
{
    // Get the map point
    Eigen::Vector2i map_point;
    worldToMap_(point, map_point);

    // Check if the point is inside the map
    if (map_point.x() < 0 || map_point.x() >= grid_size_ || map_point.y() < 0 || map_point.y() >= grid_size_)
    {
        return UNKNOWN;
    }

    // Get the value for that cell
    unsigned char value = map_data_[map_point.x()][map_point.y()];
    // Check if the cell is occupied
    if (value == 255)
    {
        return OCCUPIED;
    }
    else
    {
        return FREE;
    }
}

bool TestMap::getVoxelCenter(const Eigen::Vector2d& point, Eigen::Vector2d& voxel_center)
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
    voxel_center.x() = map_point.x()*resolution_ - map_size_/2 + resolution_/2;
    voxel_center.y() = map_point.y()*resolution_ - map_size_/2 + resolution_/2;

    return true;

}

void TestMap::worldToMap_(const Eigen::Vector2d& point, Eigen::Vector2i& map_point)
{
    // Get the map point
    map_point.x() = (point.x() + map_size_/2)/resolution_;
    map_point.y() = (point.y() + map_size_/2)/resolution_;

    // Check if the point is inside the map
    if (map_point.x() < 0 || map_point.x() >= grid_size_ || map_point.y() < 0 || map_point.y() >= grid_size_)
    {
        map_point.x() = -1;
        map_point.y() = -1;
    }

}

void TestMap::printMap()
{
    // Define a set of characters to represent different intensity levels
    const char chars[] = ".%#*+=-:O";

    // Scale the range of pixel values to match the number of characters
    const int char_count = sizeof(chars) - 1;
    const double scale = char_count / 256.0;

    // Iterate through the 2D vector and print the ASCII art
    for (int y = 0; y < grid_size_; y++) {
        for (int x = 0; x < grid_size_; x++) {
            int pixel_value = static_cast<int>(map_data_[y][x] * scale);
            char ascii_char = chars[pixel_value];
            std::cout << ascii_char;
        }
        std::cout << std::endl;
    }
}

void TestMap::printRoute(const std::vector<Eigen::Vector2d>& route)
{
    // Define a set of characters to represent different intensity levels
    const char chars[] = ".%#*+=-:O";

    // Scale the range of pixel values to match the number of characters
    const int char_count = sizeof(chars) - 1;
    const double scale = char_count / 256.0;

    // Iterate through the 2D vector and print the ASCII art
    for (int y = 0; y < grid_size_; y++) {
        for (int x = 0; x < grid_size_; x++) {
            int pixel_value = static_cast<int>(map_data_[y][x] * scale);
            // Check if the point is in the route
            bool in_route = false;
            int num_point_in_route = 0;
            for (int i = 0; i < route.size(); i++)
            {
                Eigen::Vector2i map_point;
                worldToMap_(route[i], map_point);
                if (abs(map_point.x() - x) < 0.1 && abs(map_point.y() - y) < 0.1)
                {
                    in_route = true;
                    num_point_in_route = 'x';
                    break;
                }
            }
            char ascii_char = chars[pixel_value];
            if (in_route)
            {
                ascii_char = char(num_point_in_route);
            }
            std::cout << ascii_char;
        }
        std::cout << std::endl;
    }
}


} // namespace maps
} // namespace ipp_tools
