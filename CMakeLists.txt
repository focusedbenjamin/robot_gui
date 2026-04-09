cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_srvs
  robotinfo_msgs
)

## Find OpenCV
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_srvs robotinfo_msgs
)

###################################
## Build settings
###################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare the executable
add_executable(robot_gui_node src/robot_gui_node.cpp)

## Link libraries
target_link_libraries(robot_gui_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)