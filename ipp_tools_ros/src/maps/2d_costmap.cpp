#include <iostream>

#include <ipp_tools_ros/maps/2d_costmap.h>
#include <nav2_costmap_2d/cost_values.hpp>

namespace ipp_tools
{
namespace maps
{

// Constructor
Costmap2D::Costmap2D(nav2_costmap_2d::Costmap2D* costmap)
{
    costmap_ = costmap;
}

bool Costmap2D::exists(const Eigen::Affine2d& x)
{
    // Chekcs if a cell is initialized
    unsigned int mx, my;
    if (worldToMap_(x.translation().x(), x.translation().y(), mx, my))
    {
        return costmap_->getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION;
    }
    return false;
}

// get occupancy

unsigned char Costmap2D::getVoxelState(const Eigen::Affine2d& point)
{
    // Check the state of the cell
        //     const static unsigned char OCCUPIED = 0;  // NOLINT
        // const static unsigned char FREE = 1;      // NOLINT
        // static unsigned char UNKNOWN = 2;   // NOLINT
    unsigned int mx, my;
    if (worldToMap_(point.translation().x(), point.translation().y(), mx, my))
    {
        // Use intervals from the costs
        unsigned char cost = costmap_->getCost(mx, my);
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
            return OCCUPIED;
        }
        else if (cost == nav2_costmap_2d::FREE_SPACE)
        {
            return FREE;
        }
        else if (cost < nav2_costmap_2d::MAX_NON_OBSTACLE)
        {
            return FREE;
        }
        else
        {
            return UNKNOWN;
        }
    }
    return UNKNOWN;
}

// check if a cell is traversable

bool Costmap2D::isTraversable(const Eigen::Affine2d& x)
{
    // Check if a cell is traversable
    unsigned int mx, my;
    if (worldToMap_(x.translation().x(), x.translation().y(), mx, my))
    {
        return costmap_->getCost(mx, my) < nav2_costmap_2d::MAX_NON_OBSTACLE;
    }
    return false;
}

bool Costmap2D::isPathTraversable(const Eigen::Affine2d& start, const Eigen::Affine2d& goal)
{
    // Check if a path is traversable
    // Get the direction of the path
    Eigen::Vector2d direction = (goal.translation() - start.translation()).normalized();
    // Iterate over the path
    Eigen::Vector2d point = start.translation();
    while ((point - goal.translation()).norm() > costmap_->getResolution())
    {
        point += direction*costmap_->getResolution();
        if (!isTraversable(Eigen::Affine2d(Eigen::Translation2d(point))))
        {
            return false;
        }
    }
    return true;
}

double Costmap2D::getVoxelSize()
{
    return costmap_->getResolution();
}

// get the center of a voxel from input point

bool Costmap2D::getVoxelCenter(const Eigen::Affine2d& point, Eigen::Affine2d& voxel_center)
{
    unsigned int mx, my;
    if (worldToMap_(point.translation().x(), point.translation().y(), mx, my))
    {
        double wx, wy;
        costmap_->mapToWorld(mx, my, wx, wy);
        voxel_center = Eigen::Affine2d(Eigen::Translation2d(wx, wy));
        return true;
    }
    return false;
}

bool Costmap2D::worldToMap_(double wx, double wy, unsigned int& mx, unsigned int& my)
{
    return costmap_->worldToMap(wx, wy, mx, my);
}

} // namespace maps
} // namespace ipp_tools
