/***********************************************************
 *
 * @file: base_planner.h
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
#ifndef PLANNERS_PLANNER_H
#define PLANNERS_PLANNER_H

#include <vector>

namespace ipp_tools
{
namespace planners
{

template <typename Xn>
class BasePlanner
{
public:
  /**
   * @brief Abstract class for the Base Planner object
   * Any planner should inherit from this class and
   * implement the plan() and update() functions.
   * These functions will not have any parameter or output to allow
   * abstraction. 
   * Therefore, each class should set its own parameters in the constructor
   * or self implemented setup / set/ get methods.
   *  - plan() called once to plan
   *  - update() called once per step to update the plan, data updates should
   *           be added from different methods (e.g. new observations).
   */
  BasePlanner() {}

  /**
   * @brief Destroy the Base Planner object
   */
  virtual ~BasePlanner() = default;

  /**
   * @brief Plan using the configuration in the param map
   */
  virtual void plan() = 0;

  /**
   * @brief Update the planner
   */
  virtual void update() = 0;

  /**
   * @brief Get the path
   * @return std::vector<Xn> Path
   */
  std::vector<Xn> getPath() const { return path_; }

  protected:
    std::vector<Xn> path_;
};

} // namespace planners
} // namespace ipp_tools

#endif  // PLANNERS_PLANNER_H