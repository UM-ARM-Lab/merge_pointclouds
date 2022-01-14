cmake_minimum_required(VERSION 3.5)
project(merge_pointclouds)

find_package(catkin REQUIRED roscpp sensor_msgs message_filters pcl_conversions)

catkin_package()

add_executable(interactive_icp src/interactive_icp.cpp)
target_link_libraries(interactive_icp ${catkin_LIBRARIES})
target_include_directories(interactive_icp PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(merge_pointclouds src/merge_pointclouds.cpp)
target_link_libraries(merge_pointclouds ${catkin_LIBRARIES})
target_include_directories(merge_pointclouds PUBLIC ${catkin_INCLUDE_DIRS})
