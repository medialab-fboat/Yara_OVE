cmake_minimum_required(VERSION 3.0.2)
project(fortune612_description)

find_package(catkin REQUIRED COMPONENTS
    xacro
)

catkin_package(
    CATKIN_DEPENDS xacro
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})