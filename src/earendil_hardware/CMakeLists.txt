cmake_minimum_required(VERSION 3.8)
project(earendil_hardware)

find_package(ament_cmake REQUIRED)

# This package only contains Arduino/ESP32 source files (hcode/)
# and does not build any C++ ROS 2 nodes or plugins.
# All motor/sensor communication is handled by Python nodes
# in the earendil_bot package via serial.

install(DIRECTORY arduino/
  DESTINATION share/${PROJECT_NAME}/arduino
)

install(PROGRAMS 
  script/base_gps_sender.py
  script/raspberryesp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
