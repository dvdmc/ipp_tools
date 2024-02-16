/**
 * @file   tsdf_map.h
 * @author Lukas Schmid, ASL
 *         Modified by David Morilla Cabello, Universidad de Zaragoza
 * @date   14.10.2021
 */
#ifndef MAPS_TSDF_MAP_H_
#define MAPS_TSDF_MAP_H_

#include "ipp_tools/maps/occupancy_map.h"

namespace ipp_tools
{
namespace maps
{
/**
 * @brief Basic TSDF map class templated to work with different configuration spaces.
*/
template <typename Xn>
class TSDFMap : public OccupancyMap<Xn>
{
 public:
  // constructor
  TSDFMap() = default;

  virtual ~TSDFMap() = default;

  // get the stored distance
  virtual float getVoxelDistance(const Xn &point) = 0;

  // get the stored weight
  virtual float getVoxelWeight(const Xn &point) = 0;

  // get the maximum allowed weight (return 0 if using uncapped weights)
  virtual float getMaximumWeight() = 0;
  
};

}  // end namespace maps
}  // end namespace ipp_tools

#endif  // MAPS_TSDF_MAP_H_