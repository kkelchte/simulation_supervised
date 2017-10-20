cmake_minimum_required(VERSION 2.8.3)
project(simulation_supervised_tools)


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
  bebop_msgs
)

find_package(Boost REQUIRED COMPONENTS system signals thread)

find_package(OpenCV REQUIRED)

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
include_directories(
  python
)
catkin_package()

catkin_install_python(PROGRAMS
    python/evaluate.py
    python/show_control.py
    python/show_odom_prediction.py
    python/ground_truth_listener.py
    python/console_display.py
    python/control_mapping.py
    python/show_depth_prediction.py
    python/viz_trajectories.py
    python/world_generator.py
    python/test_ounoise.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)