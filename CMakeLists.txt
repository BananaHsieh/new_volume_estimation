cmake_minimum_required(VERSION 3.0.2)
project(new_volume_estimation)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files

#Find package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
)
find_package(PCL 1.8.1 REQUIRED )  # Add visualization component
find_package(Qt5 REQUIRED Widgets)
find_package(VTK REQUIRED)

catkin_package()

link_directories( ${PCL_LIBRARY_DIRS} )
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${workspaceFolder}/include/new_volume_estimation
  ${CMAKE_CURRENT_SOURCE_DIR}/include/new_volume_estimation
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
)

add_executable(main1 
src/main1.cpp src/qnode.cpp src/main_window.cpp
include/new_volume_estimation/main_window.h 
include/new_volume_estimation/qnode.h
)
target_link_libraries(main1
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} #pcl
)

add_executable(show_pcd_file src/show_pcd_file.cpp)
target_link_libraries(show_pcd_file
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} #pcl
)