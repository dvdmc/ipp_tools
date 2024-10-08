cmake_minimum_required(VERSION 3.15)
project(ipp_tools)

if(NOT DEFINED WITH_AIRSIM)
  set(WITH_AIRSIM ON)
endif()

if(NOT DEFINED WITH_VOXBLOX)
  set(WITH_VOXBLOX ON)
endif()

if(NOT DEFINED WITH_ROS)
  set(WITH_ROS ON)
endif()

# IF WITH_VOXBLOX, we need WITH_ROS. Otherwise throw an error
if(WITH_VOXBLOX AND NOT WITH_ROS)
  message(FATAL_ERROR "WITH_VOXBLOX requires WITH_ROS")
endif()

if(WITH_AIRSIM)
  message(STATUS "Building with AirSim")
  # set this to path to AirSim root folder if you want your catkin workspace in a custom directory
  set(AIRSIM_ROOT /media/david/git/Colosseum)
  add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
  add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
  add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)
  
  set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
  set(RPC_LIB rpc) # name of .a file with lib prefix
  message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")
endif()

set(CMAKE_CXX_STANDARD 17)
# Add -o3 to compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  semantic_mapping
)

if(WITH_ROS)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
    tf_conversions
    semantic_mapping
    move_base_msgs
  )
endif()

if(WITH_VOXBLOX)
  find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  core_tools
  semantic_mapping
  move_base_msgs
  voxblox
  voxblox_ros
  voxblox_msgs
)
endif()

# Set libraries depending on optional dependencies
set(CATKIN_PROJECT_LIBRARIES ipp_tools) # Core library
if(WITH_AIRSIM)
  set(CATKIN_PROJECT_LIBRARIES ${CATKIN_PROJECT_LIBRARIES} ipp_tools_airsim) # AirSim bridge
endif()
if(WITH_ROS)
  set(CATKIN_PROJECT_LIBRARIES ${CATKIN_PROJECT_LIBRARIES} ipp_tools_ros) # ROS bridge
endif()
if(WITH_VOXBLOX)
  set(CATKIN_PROJECT_LIBRARIES ${CATKIN_PROJECT_LIBRARIES} ipp_tools_voxblox) # Voxblox map
endif()

catkin_package(
  INCLUDE_DIRS include
  ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${CATKIN_PROJECT_LIBRARIES}
  CATKIN_DEPENDS
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} # Includes voxblox
  ${EIGEN3_INCLUDE_DIRS}
)

if(WITH_AIRSIM)
include_directories(
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)
endif()

add_library(ipp_tools
src/planners/bridges/dummy_planner_bridge.cpp
src/planners/basics/poses_file_planner.cpp
src/planners/basics/random_planners.cpp
src/value/viewpoint_evaluator.cpp
src/value/semantic_value_gain_estimation.cpp
)
target_link_libraries(ipp_tools  
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

if(WITH_ROS)
add_library(ipp_tools_ros
  src/planners/bridges/move_base_planner_bridge.cpp
)
target_link_libraries(ipp_tools_ros  
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
endif()

if(WITH_AIRSIM)
add_library(ipp_tools_airsim
  src/planners/bridges/airsim_planner_bridge.cpp
)
target_link_libraries(ipp_tools_airsim  
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
endif()

if(WITH_VOXBLOX)
add_library(ipp_tools_voxblox
  src/maps/voxblox_map.cpp
)
target_link_libraries(ipp_tools_voxblox  
  ${catkin_LIBRARIES}
  Eigen3::Eigen

)
endif()

add_library(ipp_tools_tests
  src/maps/test_map.cpp
)
target_link_libraries(ipp_tools_tests  
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(test_rrt
  src/tests/test_rrt.cpp
)
target_link_libraries(test_rrt  
  ipp_tools_tests
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(test_actions
  src/tests/test_actions.cpp
)
target_link_libraries(test_actions  
  ipp_tools_tests
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)