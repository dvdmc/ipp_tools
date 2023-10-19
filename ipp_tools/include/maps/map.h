/***********************************************************
 *
 * @file: map.h
 * @breif: Contains general interface for maps in the environment
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef MAPS_MAP_H
#define MAPS_MAP_H

namespace ipp_tools
{
namespace maps
{
/**
 * @brief Basic Map class templated to work with different configuration spaces.
*/
template <typename Xn>
class Map
{
    public:
        virtual bool isTraversable(const Xn &x) = 0;
        virtual bool exists(const Xn &x) = 0;
};

} // namespace maps
} // namespace ipp_tools

#endif  // MAPS_MAP_H