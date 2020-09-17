cmake_minimum_required(VERSION 3.0.2)
project(gazebo-plugin-autonomous-actor)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    rospy
    std_msgs
)

add_library(AutonomousActorPlugin SHARED src/AutonomousActorPlugin.cc)
target_link_libraries(AutonomousActorPlugin ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Debug Version (Note to add library before target compile definition)
add_library(AutonomousActorPluginDebug SHARED src/AutonomousActorPlugin.cc)
target_compile_definitions(AutonomousActorPluginDebug PRIVATE DEBUG_)
target_link_libraries(AutonomousActorPluginDebug ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})