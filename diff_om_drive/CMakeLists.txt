cmake_minimum_required(VERSION 2.8.3)
project(diff_om_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES diff_om_drive
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  #DEPENDS system_lib
)

#*.h files from libmodbus installation directory are required in include
#and must also be referenced, so that catkin could make
add_library(${PROJECT_NAME}
    include/modbus.h
    include/modbus-rtu.h
    include/modbus-tcp.h
    include/modbus-version.h
    src/driver/om_driver_w_odom.cpp
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


add_executable(om_driver src/driver/om_driver_w_odom.cpp)
target_link_libraries (
	om_driver 
	${catkin_LIBRARIES}
	modbus
)

