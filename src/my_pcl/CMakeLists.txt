cmake_minimum_required(VERSION 3.0.2)
project(my_pcl)

find_package(catkin REQUIRED COMPONENTS

  pcl_conversions

  pcl_msgs

  pcl_ros

  sensor_msgs

)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
catkin_package()
add_executable(pcl_write src/pcl_write.cpp)

target_link_libraries(pcl_write ${catkin_LIBRARIES} ${PCL_LIBRARIES})

