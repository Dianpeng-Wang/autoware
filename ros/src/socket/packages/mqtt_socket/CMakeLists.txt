cmake_minimum_required(VERSION 2.8.3)
project(mqtt_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  vehicle_socket
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -fpermissive -lpaho-mqtt3c ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  RemoteCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  vehicle_socket
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mqtt_receiver nodes/mqtt_receiver/mqtt_receiver.cpp)
target_link_libraries(mqtt_receiver paho-mqtt3c ${catkin_LIBRARIES})
add_dependencies(mqtt_receiver
  mqtt_socket_generate_messages_cpp
)

add_executable(mqtt_sender nodes/mqtt_sender/mqtt_sender.cpp)
target_link_libraries(mqtt_sender paho-mqtt3c ${catkin_LIBRARIES})
add_dependencies(mqtt_sender
  mqtt_socket_generate_messages_cpp
)

add_executable(select_command nodes/select_command/select_command.cpp)
target_link_libraries(select_command paho-mqtt3c ${catkin_LIBRARIES})
add_dependencies(select_command
  mqtt_socket_generate_messages_cpp
)
