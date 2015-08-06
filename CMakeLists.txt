cmake_minimum_required(VERSION 2.8.3)
project(nodelet_rosbag)

find_package(catkin REQUIRED COMPONENTS nodelet rosbag_storage roscpp std_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  LIBRARIES nodelet_rosbag
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet rosbag_storage roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(nodelet_rosbag src/nodelet_rosbag.cpp)

target_link_libraries(nodelet_rosbag ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_rosbag ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS nodelet_rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
