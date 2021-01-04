cmake_minimum_required(VERSION 3.0.2)
project(target_pose)

## Find catkin macros and libraries, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    geometry_msgs
    std_msgs
    gazebo_msgs
    gazebo_ros
)

add_service_files(
  FILES
  pickplace.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  gazebo_msgs
)


catkin_package(
  LIBRARIES target_pose
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime 
    roscpp
    std_msgs
    geometry_msgs
    gazebo_msgs
    gazebo_ros
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable, the .cpp is what the executable program need
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable, keep the program after the target will be built first
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})