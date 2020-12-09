cmake_minimum_required(VERSION 3.0.2)
project(rosplan_interface_ur)

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

  std_srvs
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_planning_system
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    gazebo_msgs
    gazebo_ros
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers

    std_srvs 
    diagnostic_msgs 
    rosplan_dispatch_msgs 
    rosplan_knowledge_msgs
    rosplan_planning_system 
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable, the .cpp is what the executable program need
add_executable(RPur src/RPur.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(RPur ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable, keep the program after the target will be built first
## add_dependencies(RPur ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
