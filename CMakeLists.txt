cmake_minimum_required(VERSION 2.8.3)
project(dynamic_slam_tool)
add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  visualization_msgs
  tf
  std_msgs
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

add_library(dst_lib src/MovingObjectRemoval.cpp include/MOR/MovingObjectRemoval.h include/MOR/IncludeAll.h)
target_link_libraries(dst_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mov_i src/internal_sync_test.cpp src/component_clustering.cpp src/DBSCAN_clustering.cpp)
target_link_libraries(mov_i dst_lib)

add_executable(mov_e src/external_sync_test.cpp src/component_clustering.cpp src/DBSCAN_clustering.cpp)
target_link_libraries(mov_e dst_lib)

add_executable(mov_e2 src/external_sync_test2.cpp src/component_clustering.cpp src/DBSCAN_clustering.cpp)
target_link_libraries(mov_e2 dst_lib)
