cmake_minimum_required(VERSION 2.8.3)
project(ar_tag_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(NEEDED_PACKAGES
  ar_track_alvar
  geometry_msgs
  usb_cam
  tf
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${NEEDED_PACKAGES}
)

#######################################
## Declare ROS messages and services ##
#######################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS ${NEEDED_PACKAGES}
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ar_tag_tracking_node src/${PROJECT_NAME}/ar_tag_tracking.cpp)
target_link_libraries(ar_tag_tracking_node ${catkin_LIBRARIES})
add_dependencies(ar_tag_tracking_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############


#############
## Testing ##
#############