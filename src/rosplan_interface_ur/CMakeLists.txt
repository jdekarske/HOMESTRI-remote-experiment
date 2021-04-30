cmake_minimum_required(VERSION 3.0.2)
project(rosplan_interface_ur)

## Find catkin macros and libraries, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  target_pose
  rosplan_planning_system
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    target_pose
    roscpp
    rosplan_planning_system 
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable, the .cpp is what the executable program need
add_executable(RPGoto src/RPGoto.cpp)
add_executable(RPDrop src/RPDrop.cpp)
add_executable(RPPickup src/RPPickup.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(RPGoto ${catkin_LIBRARIES})
target_link_libraries(RPDrop ${catkin_LIBRARIES})
target_link_libraries(RPPickup ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable, keep the program after the target will be built first
## add_dependencies(RPGoto ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
