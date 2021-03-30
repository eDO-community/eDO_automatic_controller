cmake_minimum_required(VERSION 2.8.3)
project(edo_automatic_controller)

find_package(catkin REQUIRED
  COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_python_setup()


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
