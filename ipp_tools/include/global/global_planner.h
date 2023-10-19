/***********************************************************
 *
 * @file: global_planner.h
 * @breif: Contains common/commonly used nodes data strcutre
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <vector>

#include "common/nodes.h"
#include "maps/map.h"

namespace ipp_tools
{
namespace global_planner
{

template <typename Xn>
class GlobalPlanner
{
public:
  /**
   * @brief Construct a new Global Planner object
   * @param map  map of the environment to use for planning
   */
  GlobalPlanner(std::shared_ptr<maps::Map<Xn>> map) : map_(map) {}

  /**
   * @brief Create a plan from start to goal
   * @param start Start point
   * @param goal Goal point
   * @param plan Output plan
   * @return True if a plan is found
  */
  // virtual bool createPlan( TODO: To create this interface, RRT has to be changed to use conf params
  //   const Xn& start,
  //   const Xn& goal,
  //   std::vector<Xn>& plan) = 0;

private:
  std::shared_ptr<maps::Map<Xn>> map_;

};

} // namespace global_planner
} // namespace ipp_tools

#endif  // GLOBAL_PLANNER_H