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
#ifndef PLANNERS_RRT_H
#define PLANNERS_RRT_H

#include <iostream>
#include <memory>

#include "ipp_tools/common/node.h"
#include "ipp_tools/maps/map.h"
#include "ipp_tools/planners/base_planner.h"
#include "ipp_tools/samplers/random_pose_generator.hpp"

namespace ipp_tools {
namespace planners {

template <typename Xn, typename Mn, typename L>
class RRT : public BasePlanner <Xn>{
 public:
  /**
   * @brief Construct a new Global Planner object
   * @param map  map of the environment to use for planning
   */
  RRT(std::shared_ptr<maps::Map<Mn>> map, const L& limits);

  /**
   * @brief Destroy the Global Planner object
   */
  ~RRT() = default;

  /**
   * @brief Sets up the parameters for the planner
   * @param start Start point
   * @param goal Goal point
   * @param goal_tolerance Goal tolerance
   * @param max_iterations Maximum number of iterations
   * @param max_distance Maximum distance between nodes
   * @return True if the setup is successful
   *
   *
   */
  bool setup(const Xn& start, const Xn& goal, double goal_tolerance = 0.1,
             int max_iterations = 1000, double max_distance = 0.5);

  /**
   * @brief Plan using the configuration in the param map
   */
  bool plan() override;

  /**
   * @brief Update the planner
   */
  bool update() override;

  /**
   * @brief Get the path
   * @return std::vector<Xn> Path
   */
  std::vector<Xn> getPath() override;

 private:
  /**
   * @brief Sample a free point from the map
   * @return Sampled point
   * @details The point is sampled from the map using a uniform distribution
   * in the map bounds. If the sampled point is not free, it is discarded and
   * another point is sampled.
   */
  bool sampleFree_(Xn& sample);

  /**
   * @brief Find the nearest node to a given sample
   * @param sample Sample to find the nearest node to
   * @return Nearest node
   * @details The nearest node is found by iterating over the tree and
   * finding the node with the minimum distance to the sample.
   */
  const common::Node<Xn>* nearest_(const Xn& sample);

  /**
   * @brief Add a new node to the tree
   * @param sample Sample to add to the tree
   * @param nearest Nearest node to the sample
   * @details The new node is added to the tree by extending the nearest node
   * towards the sample. If the sample is too far from the nearest node, the
   * new node is added at the maximum distance from the nearest node in the
   * direction of the sample.
   */
  void addNode_(const Xn& sample, const common::Node<Xn>* nearest);

  /**
   * @brief Check if a node is close enough to the goal
   * @param node Node to check
   * @return True if the node is close enough to the goal
   */
  bool isCloseToGoal_(const common::Node<Xn>* node);

  /**
   * @brief Backtrack the tree from the goal to the start
   * @param plan Output plan
   * @param final_node Final node of the tree
   * @return True if a plan is found
   */
  bool backtrack_(std::vector<Xn>& plan, const common::Node<Xn>* final_node);

  std::shared_ptr<maps::Map<Mn>> map_;
  std::unique_ptr<samplers::RandomPoseGenerator<Xn>> sampler_;
  std::unique_ptr<common::Node<Xn>> start_, end_;
  std::vector<std::unique_ptr<common::Node<Xn>>> tree_;
  L limits_;
  double goal_tolerance_, max_distance_;
  int max_iterations_;
};

template <typename Xn, typename Mn, typename L>
RRT<Xn, Mn, L>::RRT(std::shared_ptr<maps::Map<Mn>> map, const L& limits)
    : map_(map), limits_(limits) {
  sampler_ = std::make_unique<samplers::RandomPoseGenerator<Xn>>(limits_);
}

template <typename Xn, typename Mn, typename L>
bool RRT<Xn, Mn, L>::setup(const Xn& start, const Xn& goal, double goal_tolerance,
                       int max_iterations, double max_distance) {
  this->path_.clear();  // Clear path
  start_ = std::make_unique<common::Node<Xn>>(start, 0, 0, 0, 0, nullptr);
  // This is a temporal special case
  end_ = std::make_unique<common::Node<Xn>>(goal, 0, 0, 0, 0, nullptr);
  goal_tolerance_ = goal_tolerance;
  max_iterations_ = max_iterations;
  max_distance_ = max_distance;

  // Check if the start and goal are valid
  if (!map_->isTraversable(start_->x.translation()) || !map_->isTraversable(end_->x.translation())) {
    std::cout << "Start or goal are not traversable!" << std::endl;
    return false;
  }

  return true;
}

template <typename Xn, typename Mn, typename L>
bool RRT<Xn, Mn, L>::plan() {
  // The tree holds the ownership of the nodes
  tree_.clear();
  tree_.push_back(std::move(start_));
  common::Node<Xn>* current_node = tree_.back().get();

  // Grow the tree until the goal is reached or the maximum number of
  // iterations is reached
  for (int i = 0; i < max_iterations_; i++) {
    // Sample a free point from the map
    Xn sample;
    if (!sampleFree_(sample)) {
      std::cout << "Could not sample a free point!" << std::endl;
      continue;
    }

    // Find the nearest node to the sample
    const common::Node<Xn>* nearest = nearest_(sample);
    // Check if the path between the nearest node and the sample is
    // free
    if (!map_->isPathTraversable(nearest->x.translation(), sample.translation())) {
      continue;
    }
    // Add a new node to the tree
    addNode_(sample, nearest);
    current_node = tree_.back().get();

    // Check if the new node is close enough to the goal
    if (isCloseToGoal_(current_node)) {
      // Backtrack the tree from the goal to the start
      return backtrack_(this->path_, current_node);
    }
  }
  std::cout << "Could not find a plan after " << max_iterations_ << "!"
            << std::endl;

  return false;
}

template <typename Xn, typename Mn, typename L>
bool RRT<Xn, Mn, L>::update() {
  // Does nothing
    return true;
}

template <typename Xn, typename Mn, typename L>
std::vector<Xn> RRT<Xn, Mn, L>::getPath() {
  if (this->path_.empty()) {
    std::cout << "Path is empty" << std::endl;
  }
  return this->path_;
}

template <typename Xn, typename Mn, typename L>
bool RRT<Xn, Mn, L>::sampleFree_(Xn& sample) {
  // Add a check for max iterations
  int max_iterations = 10000;
  int i = 0;
  do {
    sampler_->getPose(sample);
    i++;
  } while (!map_->isTraversable(sample.translation()) && i < max_iterations);
  return i < max_iterations;
}

template <typename Xn, typename Mn, typename L>
const common::Node<Xn>* RRT<Xn, Mn, L>::nearest_(const Xn& sample) {
  // Find the nearest node by iterating over the tree and finding the
  // node with the minimum distance to the sample
  common::Node<Xn>* nearest;
  double min_distance = std::numeric_limits<double>::max();
  for (auto& node : tree_) {
    double distance = (node->x.translation() - sample.translation()).norm();
    if (distance < min_distance) {
      min_distance = distance;
      nearest = node.get();
    }
  }

  return nearest;
}

template <typename Xn, typename Mn, typename L>
void RRT<Xn, Mn, L>::addNode_(const Xn& sample, const common::Node<Xn>* nearest) {
  // Add a new node to the tree by extending the nearest node towards
  // the sample. If the sample is too far from the nearest node, the
  // new node is added at the maximum distance from the nearest node
  // in the direction of the sample.
  Xn new_node_state;
  if ((sample.translation() - nearest->x.translation()).norm() >
      max_distance_) {
    new_node_state.translation() =
        nearest->x.translation() +
        max_distance_ *
            (sample.translation() - nearest->x.translation()).normalized();
  } else {
    new_node_state = sample;
  }

  // Create the new node
  double v = 0;  // Default in basic RRT
  double g = nearest->g +
             (new_node_state.translation() - nearest->x.translation()).norm();
  double h = (new_node_state.translation() - end_->x.translation()).norm();
  int id = tree_.size();

  std::unique_ptr<common::Node<Xn>> new_node =
      std::make_unique<common::Node<Xn>>(new_node_state, v, g, h, id, nearest);
  // Add the new node to the tree
  tree_.push_back(std::move(new_node));
}

template <typename Xn, typename Mn, typename L>
bool RRT<Xn, Mn, L>::isCloseToGoal_(const common::Node<Xn>* node) {
  // Check if a node is close enough to the goal
  return node->h < goal_tolerance_;
}

template <typename Xn, typename Mn, typename L>
bool RRT<Xn, Mn, L>::backtrack_(std::vector<Xn>& path,
                            const common::Node<Xn>* final_node) {
  // Backtrack the tree from the goal to the start
  path.clear();  // Make sure the path is empty
  const common::Node<Xn>* current_node = final_node;
  while (current_node != nullptr) {
    path.push_back(current_node->x);
    current_node = current_node->parent;
  }

  return true;
}

}  // namespace planners
}  // namespace ipp_tools

#endif  // PLANNERS_RRT_H