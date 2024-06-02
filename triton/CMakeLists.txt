cmake_minimum_required(VERSION 3.0.2)
project(triton)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros roscpp rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(thruster_plugin SHARED src/thruster_plugin.cpp)
target_link_libraries(thruster_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(velocity_plugin SHARED src/velocity_plugin.cpp)

target_link_libraries(velocity_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)
catkin_install_python(PROGRAMS
  scripts/thruster_teleop.py
  scripts/velocity_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
