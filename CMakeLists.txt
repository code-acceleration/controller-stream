cmake_minimum_required(VERSION 3.5)
project(controller_stream)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  controller_stream/joy_to_zmq.py
  controller_stream/zmq_to_joy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
