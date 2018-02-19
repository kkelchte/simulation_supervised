cmake_minimum_required(VERSION 2.8.3)
project(simulation_supervised_control)
add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	cv_bridge
	image_transport
  gazebo_ros
)

find_package(Boost REQUIRED COMPONENTS system signals thread)

find_package(OpenCV REQUIRED)

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                          include
                          python
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}	
#)
catkin_package()

catkin_install_python(PROGRAMS
    python/depth_heuristic.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(behaviourLib src/BehaviourArbitration)
target_link_libraries(behaviourLib ${OpenCV_LIBS})

add_library(ouNoise src/OUNoise)
target_link_libraries(ouNoise ${OpenCV_LIBS})

add_executable(teleop_autopilot_behaviour_arbitration src/teleop_autopilot_behaviour_arbitration.cpp)
target_link_libraries(teleop_autopilot_behaviour_arbitration
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   behaviourLib
   ouNoise
)


