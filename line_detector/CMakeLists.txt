cmake_minimum_required(VERSION 3.5)
project(line_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_detector_node src/line_detector_node.cpp)

ament_target_dependencies(line_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

target_link_libraries(line_detector_node ${OpenCV_LIBS})

install(TARGETS
  line_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
