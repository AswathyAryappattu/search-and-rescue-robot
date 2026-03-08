cmake_minimum_required(VERSION 3.0.2)
project(search_rescue)

## Find catkin and components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  actionlib
  actionlib_msgs
  tf
  tf2_ros
  cv_bridge
  image_transport
)

## System dependencies
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   VictimLocation.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   ConfirmVictim.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   SearchArea.action
# )

## Generate added messages and services with dependencies
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters
# generate_dynamic_reconfigure_options(
#   cfg/SearchRescue.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    move_base_msgs
    actionlib
    actionlib_msgs
    tf
    tf2_ros
    cv_bridge
    image_transport
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/sar_controller.py
  scripts/victim_detector.py
  scripts/path_planner.py
  scripts/communication_relay.py
  scripts/data_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files)
install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp tests
# catkin_add_gtest(${PROJECT_NAME}-test test/test_search_rescue.cpp)

## Add pytest based python tests
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(test/sar_test.test)
# endif()
