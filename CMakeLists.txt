cmake_minimum_required(VERSION 3.8)
project(pointcloud_extractor_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  roscpp
  sensor_msgs
)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_extractor_node src/pointcloud_extractor.cpp)
target_link_libraries(pointcloud_extractor_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
