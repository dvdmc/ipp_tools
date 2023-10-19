cmake_minimum_required(VERSION 3.8)
project(ipp_tools_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ipp_tools REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)


set(library_rrt_basic nav2_rrt_basic_plugin)

set(dependencies_rrt_basic
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  nav2_costmap_2d
  nav2_core
  tf2_eigen
  pluginlib
  ipp_tools
  ${Eigen_LIBRARIES}
)

add_library(${library_rrt_basic} SHARED
  src/maps/2d_costmap.cpp
  src/ros_utils.cpp
  src/rrt_nav2_interface.cpp
)

ament_target_dependencies(${library_rrt_basic}
  ${dependencies_rrt_basic}
)

target_compile_definitions(${library_rrt_basic} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS 
  ${library_rrt_basic}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${library_rrt_basic}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()