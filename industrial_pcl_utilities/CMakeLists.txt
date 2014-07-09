cmake_minimum_required(VERSION 2.8.3)
project(industrial_pcl_utilities)

find_package(catkin REQUIRED COMPONENTS pcl_msgs pcl_ros roscpp sensor_msgs tf tf_conversions)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(Boost REQUIRED COMPONENTS system)

ADD_DEFINITIONS("-std=c++0x")


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS pcl_msgs pcl_ros roscpp sensor_msgs tf tf_conversions
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## point cloud publisher node
add_executable(pcd_to_msg_node src/nodes/pcd_publisher_node.cpp)
target_link_libraries(pcd_to_msg_node ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES})

## point cloud generator node
add_executable(pcl_msg_generator_node src/nodes/point_cloud_generator_node.cpp)
target_link_libraries(pcl_msg_generator_node ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES})


