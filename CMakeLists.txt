cmake_minimum_required(VERSION 2.8.3)
project(kthfsdv-ekf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node ${catkin_LIBRARIES})
