/***********************************************************
 *
 * @file: occupancy_map.h
 * @breif: Contains general interface for occupancy maps
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef MAPS_OCCUPANCY_MAP_H
#define MAPS_OCCUPANCY_MAP_H

#include <ipp_tools/maps/map.h>

namespace ipp_tools
{
namespace maps
{
/**
 * @brief Basic Occupancy map class templated to work with different configuration spaces.
*/
template <typename Xn>
class OccupancyMap : public Map<Xn>
{
    public:
        // constructor
        OccupancyMap() = default;
    
        // states
        const static unsigned char OCCUPIED = 0;  // NOLINT
        const static unsigned char FREE = 1;      // NOLINT
        const static unsigned char UNKNOWN = 2;   // NOLINT

        // get occupancy
        virtual unsigned char getVoxelState(const Xn& point) = 0;

        // get voxel size
        virtual double getVoxelSize() = 0;

        // get the center of a voxel from input point
        virtual bool getVoxelCenter(const Xn& point, Xn& voxel_center) = 0;

};

} // namespace maps
} // namespace ipp_tools

#endif  // MAPS_OCCUPANCY_MAP_H