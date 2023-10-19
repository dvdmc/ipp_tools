#include "common/nodes.h"
#include "common/limits.h"
#include "maps/map.h"
#include "global/rrt.h"

namespace ipp_tools
{
namespace global_planner
{
    template <typename Xn, typename L>
    RRT<Xn,L>::RRT(std::shared_ptr<maps::Map<Xn>> map, const L &limits) : map_(map), limits_(limits)
    {
        sampler_ = std::make_unique<samplers::RandomPoseGenerator<Xn>>(limits_);
    }

    template <typename Xn, typename L>
    bool RRT<Xn,L>::sampleFree_(Xn &sample)
    {
        // Add a check for max iterations
        int max_iterations = 1000;
        int i = 0;
        do
        {
            sampler_->getPose(sample);
            i++;
        } while (!map_->isTraversable(sample) && i < max_iterations);
        return i < max_iterations;
    }

    template <typename Xn, typename L>
    bool RRT<Xn,L>::createPlan(const Xn &start, const Xn &goal,
        std::vector<Xn> &plan, double goal_tolerance, int max_iterations, double max_distance)
    {
        start_ = std::make_unique<common::Node<Xn>>(start, 0, 0, 0, 0, nullptr);
        // This is a temporal special case
        end_ = std::make_unique<common::Node<Xn>>(goal, 0, 0, 0, 0, nullptr); 
        goal_tolerance_ = goal_tolerance;
        max_iterations_ = max_iterations;
        max_distance_ = max_distance;

        // Check if the start and goal are valid
        if (!map_->isTraversable(start_->x) || !map_->isTraversable(end_->x))
        {
            return false;
        }

        // The tree holds the ownership of the nodes
        tree_.clear();
        tree_.push_back(std::move(start_));
        common::Node<Xn>* current_node = tree_.back().get();

        // Grow the tree until the goal is reached or the maximum number of
        // iterations is reached
        for(int i = 0; i < max_iterations_; i++)
        {
            // Sample a free point from the map
            Xn sample;
            if (!sampleFree_(sample))
            {
                continue;
            }

            // Find the nearest node to the sample
            common::Node<Xn>* nearest = nearest_(sample, nearest);

            // Add a new node to the tree
            addNode_(sample, nearest);
            current_node = tree_.back().get();

            // Check if the new node is close enough to the goal
            if (isCloseToGoal_(current_node))
            {
                // Backtrack the tree from the goal to the start
                return backtrack_(plan, current_node);
            }
        }
    }

    template <typename Xn, typename L>
    const common::Node<Xn>* RRT<Xn,L>::nearest_(const Xn &sample)
    {
        // Find the nearest node by iterating over the tree and finding the
        // node with the minimum distance to the sample
        common::Node<Xn>* nearest;
        double min_distance = std::numeric_limits<double>::max();
        for (auto &node : tree_)
        {
            double distance = (node->x.translation() - sample).norm();
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest = node.get();
            }
        }

        return nearest;
    }

    template <typename Xn, typename L>
    void RRT<Xn,L>::addNode_(const Xn &sample, const common::Node<Xn>* nearest)
    {
        // Add a new node to the tree by extending the nearest node towards
        // the sample. If the sample is too far from the nearest node, the
        // new node is added at the maximum distance from the nearest node
        // in the direction of the sample.
        Xn new_node_state;
        if ((sample.translation() - nearest->x.translation()).norm() > max_distance_)
        {
            new_node_state = nearest->x.translation() + max_distance_ * (sample - nearest->x.translation()).normalized();
        }
        else
        {
            new_node_state = sample;
        }
        
        // Create the new node
        double v = 0; // Default in basic RRT
        double g = nearest->g + (new_node_state.translation() - nearest->x.translation()).norm();
        double h = (new_node_state.translation() - end_->x.translation()).norm();
        int id = tree_.size();

        std::unique_ptr<common::Node<Xn>> new_node = std::make_unique<common::Node<Xn>>(new_node_state, v, g, h, id, nearest);
        // Add the new node to the tree
        tree_.push_back(std::move(new_node));
    }

    template <typename Xn, typename L>
    bool RRT<Xn,L>::isCloseToGoal_(const common::Node<Xn>* node)
    {
        // Check if a node is close enough to the goal
        return node->h < goal_tolerance_;
    }

    template <typename Xn, typename L>
    bool RRT<Xn,L>::backtrack_(std::vector<Xn> &plan, const common::Node<Xn>* final_node)
    {
        // Backtrack the tree from the goal to the start
        plan.clear();
        const common::Node<Xn>* current_node = final_node;
        while (current_node != nullptr)
        {
            plan.push_back(current_node->x);
            current_node = current_node->parent;
        }

        return true;
    }

} // namespace global_planner
} // namespace ipp_tools