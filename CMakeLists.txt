cmake_minimum_required(VERSION 3.0.2)
project(bmi160_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bmi160_ros
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(bmi160
  src/bmi160.cpp
  src/bmi160_interface.cpp
)

add_executable(bmi160_publisher src/bmi160_publisher.cpp)
target_link_libraries(bmi160_publisher
  bmi160
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(bmi160_calibration_node src/bmi160_calibration_node.cpp)
target_link_libraries(bmi160_calibration_node
  bmi160
  ${catkin_LIBRARIES}
  yaml-cpp
  ${EIGEN3_LIBRARIES}
)
