cmake_minimum_required(VERSION 2.8.3)
project(ares_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -o3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)

set(G2O_LIBS
  g2o_solver_slam2d_linear g2o_types_slam2d g2o_types_sba g2o_core g2o_interface
  g2o_stuff g2o_types_sclam2d g2o_parser g2o_types_data
)

catkin_package(
)

include_directories(
  include/ares_slam
  "/usr/include/eigen3"
  ${catkin_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
)

add_executable(ares_slam_node
  src/node.cpp
  src/ares_slam_ros.cpp
  src/map_builder.cpp
  src/correlative_scan_matcher.cpp
  src/gauss_newten_scan_matcher.cpp
  src/correlative_grid.cpp
  src/occupancy_grid_map.cpp
  src/probability_grid_map.cpp
)

target_link_libraries(ares_slam_node
  ${catkin_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${G2O_LIBS}
)

install(TARGETS ares_slam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
