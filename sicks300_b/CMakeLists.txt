cmake_minimum_required(VERSION 2.8.3)
project(sicks300_b)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serialcomm_s300B
  CATKIN_DEPENDS roscpp tf sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(serialcomm_s300B
  src/serialcomm_s300B.cpp)

#target_link_libraries(serialcomm_s300 ${catkin_LIBRARIES})

add_executable(sick300_driver_B src/sicks300B.cpp)

target_link_libraries(sick300_driver_B
  serialcomm_s300B
  ${catkin_LIBRARIES}
)

install(TARGETS serialcomm_s300B sick300_driver_B
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)





