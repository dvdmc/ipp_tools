cmake_minimum_required(VERSION 3.8)
project(ipp_tools)
set(CMAKE_VERBOSE_MAKEFILE ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

set(dependencies
${Eigen_LIBRARIES}
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/maps/test_map.cpp
  src/global/rrt.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(rrt_test src/tests/rrt_test.cpp)

target_link_libraries(rrt_test
  ${library_name}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()