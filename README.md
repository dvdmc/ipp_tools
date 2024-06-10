# Informative Path Planning (IPP) tools

This is intended to be a generic library for widely used methods in Informative Path Planning or Active Perception in robotics.
It is designed with IPP and AP in mind but it can potentially be used for simple path planning.
The structure is intended to be agnostic to the configuration space with particular definitions for 2D and 3D.

## Dependencies 

This code depends on the [semantic_mapping](https://github.com/dvdmc/semantic_mapping) library. 
In the future, we plan on removing this dependency by creating wrappers around the library.
The library also depends on Eigen3. The bridges have different dependencies that can be 
configured in the `CMakeLists.txt` for using Airsim, Voxblox, or ROS. Some modules might become
unavailable if the dependencies are deactivated.

## Installation

Clone the repository with:

```
git clone https://github.com/dvdmc/ipp_tools
```

Currently, this code is intended to be used as a library or examples. You can include this library
in other projects. The easier way is using ROS and compiling with:

```
catkin build ipp_tools
```

## Structure
- `ipp_tools/common/`: definition of action spaces, map limits, cameras (parameters, frustum checks, etc.), generic node for planning. When possible, the number of dimensions can be configured to 2 or 3. 
- `ipp_tools/samplers`: pose sampling algorithms.
- `ipp_tools/value`: information value gain estimation algorithms. Some of them depend on the `semantic_mapping` package by using the information structure and fusion for semantic measurements.
- `ipp_tools/planners`: includes different planners and bridges to send poses to different simulators, and robot planning frameworks (e.g., navigation stack).
- `ipp_tools/maps`: abstract classes that work as wrappers for generic maps and external libraries. They include different functions used in the planners that every map should implement.



