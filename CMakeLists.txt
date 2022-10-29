cmake_minimum_required(VERSION 3.0.2)
project(velodynescan_to_pc2)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(
    catkin REQUIRED
    roscpp
    rosbag
    velodyne_msgs
)
find_package(PCL)
find_package(velodyne_pointcloud)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
    CATKIN_DEPENDS velodyne_pointcloud
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${velodyne_pointcloud_INCLUDE_DIRS}
)

add_executable(velodynescan_to_pc2
    src/velodynescan_to_pc2.cpp
)
target_link_libraries(velodynescan_to_pc2
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${velodyne_pointcloud_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
