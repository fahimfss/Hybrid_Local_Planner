cmake_minimum_required(VERSION 2.8.3)
project(tc_gazebo_custom_plugins)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


# For model_push_plugin plugin
add_library(model_push_plugin src/model_push.cc)
target_link_libraries(model_push_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} -lprotobuf)


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)