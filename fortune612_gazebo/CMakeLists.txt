cmake_minimum_required(VERSION 3.0.2)
project(fortune612_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Set policy for CMake 3.1+, FindBoost.cmake dependency of gazebo uses quoted
# variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros
  wave_gazebo
  roscpp
  rospy
  std_msgs
  xacro
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
	gazebo_dev
	gazebo_ros
	wave_gazebo
	roscpp
	rospy
	std_msgs
	xacro
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Run xacro to generate world files
xacro_add_files(
  worlds/ocean.world.xacro
  worlds/ocean_fixed_cam.world.xacro
  INSTALL DESTINATION worlds
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#install(DIRECTORY config/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

## Declare a C++ library

###########
# Install #
###########
install(TARGETS usv_gazebo_wind_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)