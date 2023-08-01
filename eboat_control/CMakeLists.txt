cmake_minimum_required(VERSION 3.0.2)
project(eboat_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros
  roscpp
  std_msgs
)

add_executable(lidarController src/lidarController.cc)
target_link_libraries(lidarController ${catkin_LIBRARIES})


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

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
    INCLUDE_DIRS include
    LIBRARIES eboat_gazebo_plugins
    CATKIN_DEPENDS gazebo_dev gazebo_ros roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

## Propulsion Control Interface for ROS
add_library(PropulsorControllerPlugin src/propulsorController.cc)
target_link_libraries(PropulsorControllerPlugin ${GAZEBO_LIBRARIES}
                      ${roscpp_LIBRARIES}
                      ${std_msgs_LIBRARIES}
                      ${Boost_LIBRARIES}
                     )

## Rudder Control Interface for ROS
add_library(RudderControllerPlugin src/rudderController.cc)
target_link_libraries(RudderControllerPlugin ${GAZEBO_LIBRARIES}
                      ${roscpp_LIBRARIES}
                      ${std_msgs_LIBRARIES}
                      ${Boost_LIBRARIES}
                     )

## Sail Control Interface for ROS
add_library(SailControllerPlugin src/sailController.cc)
target_link_libraries(SailControllerPlugin ${GAZEBO_LIBRARIES}
                      ${roscpp_LIBRARIES}
                      ${std_msgs_LIBRARIES}
                      ${Boost_LIBRARIES}
                     )
                     
## Mission Control
add_library(MissionControlPlugin src/missionControl.cc)
target_link_libraries(MissionControlPlugin ${GAZEBO_LIBRARIES}
                      ${roscpp_LIBRARIES}
                      ${std_msgs_LIBRARIES}
                      ${Boost_LIBRARIES}
                     )
