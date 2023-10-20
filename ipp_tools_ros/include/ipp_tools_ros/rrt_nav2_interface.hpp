/***********************************************************
 *
 * @file: nav2_interface.h
 * @breif: Creates a common interface for the nav2 stack
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef ROS_RRT_NAV2_INTERFACE_H
#define ROS_RRT_NAV2_INTERFACE_H

#include <string>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "maps/2d_costmap.h"
#include "global/rrt.h"
#include "common/limits.h"
#include "ipp_tools_ros/ros_utils.hpp"
#include "ipp_tools_ros/visualizers/path_visualizer.hpp"

namespace ipp_tools
{
namespace nav2_interface
{

    class RRTBasic : public nav2_core::GlobalPlanner
    {
        public:
            /**
             * @brief Construct a new RRT Planner object
             */
            RRTBasic() = default;

            /**
             * @brief Destroy the RRT Planner object
             */
            ~RRTBasic() = default;

            /**
             * @brief Configure the planner
             * @param parent Lifecycle node pointer
             * @param name Name of the planner
             * @param tf TF buffer
             * @param costmap_ros Costmap ROS wrapper
             */
            void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

            /**
             * @brief Plugin activate
             */
            void activate();

            /**
             * @brief Plugin deactivate
             */
            void deactivate();

            /**
             * @brief Plugin cleanup
             */
            void cleanup();

            /**
             * @brief Create a plan from a starting pose to an ending pose
             * @param start Starting pose
             * @param goal Ending pose
             * @return nav_msgs::msg::Path Path from start to goal
             */
            nav_msgs::msg::Path createPlan(
                const geometry_msgs::msg::PoseStamped & start,
                const geometry_msgs::msg::PoseStamped & goal);

        private:


            // TF buffer
            std::shared_ptr<tf2_ros::Buffer> tf_;
            // Lifecycle node
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

            // ROS global costmap
            nav2_costmap_2d::Costmap2D* costmap_ros_;

            // Map
            std::shared_ptr<maps::Costmap2D> map_;
            
            // Planner. Initialized in configure
            std::unique_ptr<global_planner::RRT<Eigen::Affine2d, common::Limits2D>> planner_;

            // Visualizer. Initialized in configure
            std::unique_ptr<visualizers::PathVisualizer> visualizer_;

            // The global frame of the costmap
            std::string global_frame_, name_;

            // Save the last path
            nav_msgs::msg::Path last_path_;
    };

} // namespace nav2_interface
} // namespace ipp_tools


#endif // ROS_RRT_NAV2_INTERFACE_H