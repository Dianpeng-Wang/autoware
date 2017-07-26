cmake_minimum_required(VERSION 2.8.12)
project(camera_lidar_fuser)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include(FindPkgConfig)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
	FIND_PACKAGE(catkin REQUIRED COMPONENTS
	  cv_bridge
	  image_transport
	  roscpp
	  sensor_msgs
	  std_msgs
	  autoware_msgs
	  tf
	  jsk_recognition_msgs
	  jsk_rviz_plugins
	)
elseif ("${ROS_VERSION}" MATCHES "(kinetic)")
	FIND_PACKAGE(catkin REQUIRED COMPONENTS
	  cv_bridge
	  image_transport
	  roscpp
	  std_msgs
	  autoware_msgs
	  sensor_msgs
	  tf
	  jsk_recognition_msgs
	  jsk_rviz_plugins
)
endif()

FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
	catkin_package(CATKIN_DEPENDS 
	  std_msgs geometry_msgs autoware_msgs
	)
elseif ("${ROS_VERSION}" MATCHES "(kinetic)")
	catkin_package(CATKIN_DEPENDS
	  std_msgs geometry_msgs autoware_msgs
	)
endif()

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(camera_lidar_fuser 
	src/camera_lidar_fuser.cpp
)

add_dependencies(camera_lidar_fuser
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries( camera_lidar_fuser PRIVATE
	${OpenCV_LIBRARIES}
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)