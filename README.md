# Informative Path Planning (IPP) tools

This is intended to be a generic library for widely used methods in Informative Path Planning or Active Perception in robotics. It is designed with IPP and AP in mind but it can potentially be used for simple path planning.

The structure is intended to be agnostic to the configuration space with particular definitions for 2D and 3D.

## Integration with your code

Right now, this is plain code that has to be manually integrated. In the future, a proper library will be built around it.

## Requirements

The executables in this code are created for ROS 2. There are dependencies on the Eigen3 library and the navigation2 package.

## Simple tests

Right now only RRT is implemented. You can test it by executing the following command:

`ros2 run ipp_tools rrt_test`

If you want to test the plugin for the navigation2 package, you should execute:

```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=./src/ipp_tools/ipp_tools_ros/params/nav2_params.yaml headless:=False
```
