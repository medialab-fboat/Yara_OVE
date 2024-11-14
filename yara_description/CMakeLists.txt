cmake_minimum_required(VERSION 3.0.2)
project(yara_description)

find_package(catkin REQUIRED COMPONENTS
    xacro
)

catkin_package(
    CATKIN_DEPENDS xacro
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#xacro_add_files(
#    urdf/eboat_base.urdf.xacro
#    ${XACRO_INORDER} INSTALL DESTINATION urdf
#)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})