cmake_minimum_required(VERSION 3.0.2)
project(planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Path.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs
 )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES planner
  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${OpenCV_INCLUDE_DIRS}
)


add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(planner planner_generate_messages_cpp)




