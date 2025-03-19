cmake_minimum_required(VERSION 3.8)
project(vff_avoidance)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(vff_avoidance src/avoidance_vff_main.cpp)
ament_target_dependencies(vff_avoidance rclcpp sensor_msgs geometry_msgs visualization_msgs)

install(TARGETS
  vff_avoidance
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
