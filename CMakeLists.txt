cmake_minimum_required(VERSION 2.8.3)
project(soft_robot_project)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs image_transport cv_bridge message_generation)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opportunistic_link
  CATKIN_DEPENDS rospy std_msgs sensor_msgs image_transport cv_bridge message_runtime
  DEPENDS system_lib rt OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O3")

# Color to binary image node
add_executable(soft_robot_tracker src/soft_robot_tracker.cpp)
add_dependencies(soft_robot_tracker ${catkin_EXPORTED_TARGETS})
target_link_libraries(soft_robot_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
