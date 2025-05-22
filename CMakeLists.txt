cmake_minimum_required(VERSION 3.5)
project(ros2-zmq-bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

include_directories(${ZMQ_INCLUDE_DIRS})

add_executable(joy_to_zmq src/joy_to_zmq.cpp)
ament_target_dependencies(joy_to_zmq rclcpp sensor_msgs)
target_link_libraries(joy_to_zmq ${ZMQ_LIBRARIES})

add_executable(zmq_to_joy src/zmq_to_joy.cpp)
add_executable(ros2_zmq_bridge src/ros2_zmq_bridge.cpp)
ament_target_dependencies(ros2_zmq_bridge rclcpp)
target_link_libraries(ros2_zmq_bridge ${ZMQ_LIBRARIES})
ament_target_dependencies(zmq_to_joy rclcpp sensor_msgs)
target_link_libraries(zmq_to_joy ${ZMQ_LIBRARIES})

install(TARGETS joy_to_zmq zmq_to_joy ros2_zmq_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
