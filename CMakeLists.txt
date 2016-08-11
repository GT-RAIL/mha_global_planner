cmake_minimum_required(VERSION 2.8.3)
project(mha_global_planner)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  sbpl_catkin
  geometry_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(CMAKE_BUILD_TYPE Debug)

generate_dynamic_reconfigure_options(cfg/MhaGlobalPlanner.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mha_global_planner
  CATKIN_DEPENDS dynamic_reconfigure roscpp sbpl_catkin geometry_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(mha_global_planner
  src/avoid_square_heuristic.cpp
  src/demo_path_distance_heuristic.cpp
  src/euclidean_distance_heuristic.cpp
  src/mha_global_planner.cpp
  src/base_heuristic.cpp
  src/hashable_cell.cpp)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${SBPL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
add_definitions(-DROS)

target_link_libraries(mha_global_planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SBPL_LIBRARIES}
)

add_dependencies(mha_global_planner ${PROJECT_NAME}_gencfg)
add_dependencies(mha_global_planner ${catkin_EXPORTED_TARGETS})
