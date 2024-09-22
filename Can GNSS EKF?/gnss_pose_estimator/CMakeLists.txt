cmake_minimum_required(VERSION 3.5)
project(gnss_pose_estimator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(gnss_pose_estimator src/gnss_pose_estimator.cpp)

ament_target_dependencies(gnss_pose_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS
  gnss_pose_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
