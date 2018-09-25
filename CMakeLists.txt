cmake_minimum_required(VERSION 2.8.3)
project(kthfsdv-ekf)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  roscpp
  cmake_modules
)

find_package(Eigen REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node ${catkin_LIBRARIES})
