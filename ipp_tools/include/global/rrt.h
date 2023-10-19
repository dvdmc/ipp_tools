/***********************************************************
 *
 * @file: rrt.h
 * @breif: Contains the basic RRT implementation
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef GLOBAL_RRT_H
#define GLOBAL_RRT_H

#include <memory>

#include "common/nodes.h"
#include "maps/map.h"
#include "global/global_planner.h"
#include "samplers/random_pose_generator.hpp"

namespace ipp_tools
{
namespace global_planner
{

template <typename Xn, typename L>
class RRT : public GlobalPlanner<Xn>
{
public:
  /**
   * @brief Construct a new Global Planner object
   * @param map  map of the environment to use for planning
   */
  RRT(std::shared_ptr<maps::Map<Xn>> map, const L &limits);

  /**
   * @brief Destroy the Global Planner object
   */
  ~RRT() = default;

  /**
   * @brief Create a plan from start to goal
   * @param start Start point
   * @param goal Goal point
   * @param plan Output plan
   * @return True if a plan is found
  */
  bool createPlan(
    const Xn& start,
    const Xn& goal,
    std::vector<Xn>& plan, double goal_tolerance = 0.1, int max_iterations = 1000, double max_distance = 0.5);
    
private:

  /**
   * @brief Sample a free point from the map
   * @return Sampled point
   * @details The point is sampled from the map using a uniform distribution
   * in the map bounds. If the sampled point is not free, it is discarded and
   * another point is sampled.
  */
  bool sampleFree_(Xn &sample);

  /**
   * @brief Find the nearest node to a given sample
   * @param sample Sample to find the nearest node to
   * @return Nearest node
   * @details The nearest node is found by iterating over the tree and
   * finding the node with the minimum distance to the sample.
  */
  const common::Node<Xn>* nearest_(const Xn &sample);

  /**
   * @brief Add a new node to the tree
   * @param sample Sample to add to the tree
   * @param nearest Nearest node to the sample
   * @details The new node is added to the tree by extending the nearest node
   * towards the sample. If the sample is too far from the nearest node, the
   * new node is added at the maximum distance from the nearest node in the
   * direction of the sample.
  */
  void addNode_(const Xn &sample, const common::Node<Xn>* nearest);

  /**
   * @brief Check if a node is close enough to the goal
   * @param node Node to check
   * @return True if the node is close enough to the goal
  */
  bool isCloseToGoal_( const common::Node<Xn>* node);

  /**
   * @brief Backtrack the tree from the goal to the start
   * @param plan Output plan
   * @param final_node Final node of the tree
   * @return True if a plan is found
  */
  bool backtrack_(std::vector<Xn> &plan, const common::Node<Xn>* final_node);

  std::shared_ptr<maps::Map<Xn>> map_;
  std::unique_ptr<samplers::RandomPoseGenerator<Xn>> sampler_;
  std::unique_ptr<common::Node<Xn>> start_, end_;
  std::vector<std::unique_ptr<common::Node<Xn>>> tree_;
  L limits_;
  double goal_tolerance_, max_distance_;
  int max_iterations_;
};

} // namespace global_planner
} // namespace ipp_tools

#endif  // GLOBAL_RRT_H