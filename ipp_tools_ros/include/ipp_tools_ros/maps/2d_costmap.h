/***********************************************************
 *
 * @file: 2d_costmap.h
 * @breif: Contains specific implementation for 2D costmaps
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef ROS_MAPS_2D_COSTMAP_H
#define ROS_MAPS_2D_COSTMAP_H

#include <memory>

#include <Eigen/Dense>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <nav2_msgs/msg/costmap.hpp>

#include <maps/occupancy_map.h>

namespace ipp_tools
{
namespace maps
{
/**
 * @brief Specific implementation of the 2D costmap.
*/
class Costmap2D : public OccupancyMap<Eigen::Affine2d>
{
    public:
        /**
         * @brief Construct a new Costmap2D object
         * @param costmap Costmap2D pointer
         */
        Costmap2D(nav2_costmap_2d::Costmap2D* costmap);

        /**
         * @brief Destroy the Costmap2D object
         */
        static Costmap2D fromROSMsg(const nav_msgs::msg::OccupancyGrid& msg);

        /**
         * @brief Check if a cell is initialized
         * @param x Input point
         * @return True if the cell is initialized
         */
        bool exists(const Eigen::Affine2d &x);

        /**
         * @brief Get the occupancy of a cell
         * @param point Input point
         * @return Occupancy of the cell
         * @details The occupancy is defined as:
         *    OCCUPIED = 0;
         *    FREE = 1;    
         *    UNKNOWN = 2; 
        */
        unsigned char getVoxelState(const Eigen::Affine2d& point);

        /**
         * @brief Check if a cell is traversable
         * @param x Input point
         * @return True if the cell is traversable
         */
        bool isTraversable(const Eigen::Affine2d &x);
        
        /**
         * @brief Check if a path is traversable
         * @param x Input point
         * @return True if the path is traversable
         */
        bool isPathTraversable(const Eigen::Affine2d &start, const Eigen::Affine2d &goal);
        

        /**
         * @brief Get the voxel size
         * @return Voxel size
         */
        double getVoxelSize();

        /**
         * @brief Get the center of a voxel from input point
         * @param point Input point
         * @param voxel_center Output voxel center
         * @return True if the voxel center is found
         */
        bool getVoxelCenter(const Eigen::Affine2d& point, Eigen::Affine2d& voxel_center);

    private:

        /**
         * @brief Transform from world coordinates to map coordinates
         * @param wx World x coordinate
         * @param wy World y coordinate
         * @param mx Map x coordinate
         * @param my Map y coordinate
         * @return True if the transformation is successful
         */
        bool worldToMap_(double wx, double wy, unsigned int& mx, unsigned int& my);

        // Private costmap
        nav2_costmap_2d::Costmap2D* costmap_;

};

} // namespace maps
} // namespace ipp_tools

#endif  // ROS_MAPS_2D_COSTMAP_H