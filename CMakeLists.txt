cmake_minimum_required(VERSION 3.0.2)
project(feature_extraction)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  livox_ros_driver
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES feature_extraction
#  CATKIN_DEPENDS pcl_ros roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_executable(feature_extract src/feature_extract.cpp src/feature_extract_node.cpp)
target_link_libraries(feature_extract ${catkin_LIBRARIES})
