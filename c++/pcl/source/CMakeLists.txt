cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cloud_viewer)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

add_executable (cloud_viewer_multi cloud_viewer_multi.cpp)
target_link_libraries (cloud_viewer_multi ${PCL_LIBRARIES})

add_executable (cloud_merge cloud_merge.cpp)
target_link_libraries (cloud_merge ${PCL_LIBRARIES})

add_executable (lidar_calibrator lidar_calibrator_test.cpp)
target_link_libraries (lidar_calibrator ${PCL_LIBRARIES})

add_executable (icp_pointcloud icp_pointcloud.cpp)
target_link_libraries (icp_pointcloud ${PCL_LIBRARIES})

add_executable (icp_pointcloud_2 icp_pointcloud_2.cpp)
target_link_libraries (icp_pointcloud_2 ${PCL_LIBRARIES})

add_executable (ndt_pointcloud ndt_pointcloud.cpp)
target_link_libraries (ndt_pointcloud ${PCL_LIBRARIES})