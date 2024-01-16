cmake_minimum_required(VERSION 3.15)
project(ipp_tools)

set(CMAKE_CXX_STANDARD 17)
# Add -o3 to compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Find system libraries
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(test_map
  src/maps/test_map.cpp
)
target_link_libraries(test_map  
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(test_rrt
  src/tests/test_rrt.cpp
)
target_link_libraries(test_rrt  
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)