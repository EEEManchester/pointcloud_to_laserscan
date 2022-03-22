cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_laserscan)

add_compile_options(-std=c++11)

set(CMAKE_CONFIGURATION_TYPES "Release")


find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros  
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/slicer.cfg
#   cfg/DynReconf2.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laserscan_to_pointcloud pointcloud_to_laserscan
  CATKIN_DEPENDS laser_geometry message_filters nodelet roscpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(laserscan_to_pointcloud src/laserscan_to_pointcloud_nodelet.cpp)
#target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})

#add_executable(laserscan_to_pointcloud_node src/laserscan_to_pointcloud_node.cpp)
#target_link_libraries(laserscan_to_pointcloud_node laserscan_to_pointcloud ${catkin_LIBRARIES} )

#add_library(pointcloud_to_laserscan src/pointcloud_to_laserscan_nodelet.cpp)
#target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
#target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_tom.cpp)
target_link_libraries(pointcloud_to_laserscan_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_to_laserscan_node ${PROJECT_NAME}_gencfg)




if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
