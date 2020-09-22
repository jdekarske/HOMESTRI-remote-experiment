project(targetpose)

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
  LIBRARIES
  INCLUDE_DIRS
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

include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable, the .cpp is what the executable program need
add_executable(targetpose src/targetpose.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(targetpose ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable, keep the program after the target will be built first
add_dependencies(targetpose targetpose)