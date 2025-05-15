cmake_minimum_required(VERSION 3.8)
project(detect_removal)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(detect_removal src/detect_removal.cpp)
ament_target_dependencies(detect_removal
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  std_srvs
)

install(TARGETS detect_removal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
