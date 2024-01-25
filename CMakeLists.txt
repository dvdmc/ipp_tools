cmake_minimum_required(VERSION 3.15)
project(ipp_tools)

if(NOT DEFINED WITH_AIRSIM)
  set(WITH_AIRSIM ON)
endif()

if(WITH_AIRSIM)
  message(STATUS "Building with AirSim")
  # set this to path to AirSim root folder if you want your catkin workspace in a custom directory
  set(AIRSIM_ROOT /media/david/AirSim)
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

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
  ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ipp_tools ipp_tools_tests
  CATKIN_DEPENDS
  DEPENDS Eigen3
)

if(WITH_AIRSIM)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS}
)
else()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
endif()

if(WITH_AIRSIM)
add_library(ipp_tools
  src/planners/basics/poses_file_planner.cpp
  src/planners/bridges/airsim_planner_bridge.cpp
)
target_link_libraries(ipp_tools  
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

else()
add_library(ipp_tools
  src/planners/basics/poses_file_planner.cpp
)
target_link_libraries(ipp_tools  
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