/***********************************************************
 *
 * @file: test_map.h
 * @breif: Test map for RRT algorithm. It is a 2D map with a circle in the middle.
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef MAPS_TEST_MAP_H
#define MAPS_TEST_MAP_H
#include <iostream>

#include <Eigen/Dense>

#include <ipp_tools/maps/occupancy_map.h>

namespace ipp_tools
{
namespace maps
{
/**
 * @brief Basic Map class templated to work with different configuration spaces.
*/
class TestMap : public OccupancyMap<Eigen::Affine2d>
{
    public:
        TestMap(double resolution = 0.1);

        // get occupancy
        unsigned char getVoxelState(const Eigen::Affine2d& point);

        // get voxel size
        inline double getVoxelSize() { return resolution_; }

        // get the center of a voxel from input point
        bool getVoxelCenter(const Eigen::Affine2d& point, Eigen::Affine2d& voxel_center);

        bool isTraversable(const Eigen::Affine2d& point);

        bool isPathTraversable(const Eigen::Affine2d& start, const Eigen::Affine2d& goal);
        
        bool exists(const Eigen::Affine2d& point);

        void printMap();

        void printRoute(const std::vector<Eigen::Affine2d>& route);

    private:
        void worldToMap_(const Eigen::Affine2d& point, Eigen::Vector2i& map_point);

        // The actual map structure. It is 10x10 meters with a circle in the middle
        // with radius 1 meter
        double map_size_ = 10; //m
        int grid_size_;
        double resolution_;
        double map_center_;
        int grid_center_;
        float circle_size_ = 1; //m
        int circle_radius_;

        // Map data
        std::vector<std::vector<unsigned char>> map_data_;
};

} // namespace maps
} // namespace ipp_tools

#endif  // MAPS_TEST_MAP_H