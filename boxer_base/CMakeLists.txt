cmake_minimum_required(VERSION 2.8.3)
project(boxer_base)

find_package(catkin REQUIRED COMPONENTS
  cpr_common_msgs controller_manager cpr_common_srvs diagnostic_updater geometry_msgs hardware_interface boxer_msgs
  realtime_tools roscpp rospy rosserial_server sensor_msgs std_msgs teleop_twist_joy
  topic_tools rosgraph_msgs urdf image_transport cv_bridge urg_node)

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(OpenCV)

catkin_package()

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra")

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

add_executable(boxer_node
  src/boxer_base.cpp
  src/boxer_diagnostic_updater.cpp
  src/boxer_hardware.cpp)
target_link_libraries(boxer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(boxer_node boxer_msgs_generate_messages_cpp rosserial_msgs_generate_messages_cpp)

add_executable(camera_focus src/camera_focus_node.cpp)
target_link_libraries(camera_focus ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_simple_joy_node src/simple_joy_node.cpp)
target_link_libraries(${PROJECT_NAME}_simple_joy_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_simple_joy_node
  PROPERTIES OUTPUT_NAME simple_joy_node PREFIX "")
add_dependencies(${PROJECT_NAME}_simple_joy_node boxer_msgs_generate_messages_cpp)

install(TARGETS boxer_node ${PROJECT_NAME}_simple_joy_node camera_focus
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/net_logger
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/calibrate_compass
  scripts/compute_calibration
  scripts/motors_ping_pong
  scripts/calibrate_stereo
  scripts/camera_set_focus
  scripts/update_camera_params
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  # @todo Enable this test when CORE-5634 gets done and solves the issue with the /run/... files
  #find_package(roslaunch)
  #roslaunch_add_file_check(launch/base.launch)

  find_package(roslint)
  roslint_cpp()
  roslint_python()
  roslint_add_test()
endif()
