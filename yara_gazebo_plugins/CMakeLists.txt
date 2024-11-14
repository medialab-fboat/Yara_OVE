cmake_minimum_required(VERSION 3.0.2)
project(yara_gazebo_plugins)

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
    LIBRARIES yara_gazebo_plugins
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

## Windvane/WindSock lift & drag Plugin
add_library(WindsockPlugin src/WindsockPlugin.cc)
target_link_libraries(WindsockPlugin ${GAZEBO_LIBRARIES} 
                      #${roscpp_LIBRARIES} 
                      ${Boost_LIBRARIES}
                     )

## Atmosferic Control
add_library(AtmosfericControlPlugin src/atmosfericControl.cc)
target_link_libraries(AtmosfericControlPlugin ${GAZEBO_LIBRARIES}
                      ${roscpp_LIBRARIES} 
                      ${Boost_LIBRARIES}
                     )

## Lift and drag forces
add_library(LiftDragForces src/liftDragForces.cc)
target_link_libraries(LiftDragForces ${GAZEBO_LIBRARIES}
                      ${Boost_LIBRARIES}
                     )

## Lift and drag forces
add_library(SailLiftDragForces src/sailLiftDragForces.cc)
target_link_libraries(SailLiftDragForces ${GAZEBO_LIBRARIES}
                      ${Boost_LIBRARIES}
                    )

## Under water Lift and drag forces
add_library(UnderWaterLiftDrag src/underWaterLiftDrag.cc)
target_link_libraries(UnderWaterLiftDrag ${GAZEBO_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )