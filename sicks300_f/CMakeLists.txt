cmake_minimum_required(VERSION 2.8.3)
project(sicks300_f)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serialcomm_s300F
  CATKIN_DEPENDS roscpp tf sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(serialcomm_s300F
  src/serialcomm_s300F.cpp)

#target_link_libraries(serialcomm_s300 ${catkin_LIBRARIES})

add_executable(sick300_driver_F src/sicks300F.cpp)

target_link_libraries(sick300_driver_F
  serialcomm_s300F
  ${catkin_LIBRARIES}
)

install(TARGETS serialcomm_s300F sick300_driver_F
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)





