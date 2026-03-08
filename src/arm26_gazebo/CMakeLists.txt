cmake_minimum_required(VERSION 3.4)
project(arm26_gazebo)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  joint_trajectory_controller
  arm26_description
  position_controllers
  robot_state_publisher
  roscpp
  std_msgs
  xacro
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    controller_manager
    effort_controllers
    gazebo_ros
    gazebo_ros_control
    joint_state_controller
    joint_trajectory_controller
    arm26_description
    position_controllers
    robot_state_publisher
    roscpp
    std_msgs
    xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY launch config world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
