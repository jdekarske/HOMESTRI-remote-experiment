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
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ executable, the .cpp is what the executable program need
add_executable(targetpose src/targetpose.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(targetpose ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable, keep the program after the target will be built first
add_dependencies(targetpose targetpose)