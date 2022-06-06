cmake_minimum_required(VERSION 2.8.3)
project(boxer_base)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/start-ros-bridge
  scripts/start-autonomy-bridge
  scripts/start-fleet-bridge
  scripts/start-platform-bridge
  scripts/restart-adaptor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  # @todo Enable this test when CORE-5634 gets done and solves the issue with the /run/... files
  #find_package(roslaunch)
  #roslaunch_add_file_check(launch/base.launch)
endif()
