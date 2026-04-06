cmake_minimum_required(VERSION 2.8.3)
project(boxer_bringup)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES boxer_bringup
#  CATKIN_DEPENDS robot_upstart boxer_base
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/install
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
