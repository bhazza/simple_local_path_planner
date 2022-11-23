cmake_minimum_required(VERSION 3.0.2)
project(simple_local_path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  costmap_2d
)

find_package(GTest REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/simple_local_path_planner_config.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_local_path_planner
#  CATKIN_DEPENDS nav_core roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/simple_local_path_planner.cpp
  src/simple_local_path_planner_ros.cpp
)
# Build dynamic reconfigure headers before library
add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
  )


#############
## Testing ##
#############

catkin_add_gtest(unit_tests test/test_local_path_planner.cpp)
target_link_libraries(unit_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

