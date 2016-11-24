cmake_minimum_required(VERSION 2.8.3)
project(manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  genmsg 
  actionlib_msgs 
  actionlib
  message_generation
)


add_action_files(DIRECTORY action FILES PickUpObject.action)

add_message_files(
  FILES
  Manipulation.msg
)

add_service_files(
  FILES
  CarryObject.srv
  DropObject.srv
  PickupObject.srv
  PickupTrap.srv
  PlaceObject.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES manipulation
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


