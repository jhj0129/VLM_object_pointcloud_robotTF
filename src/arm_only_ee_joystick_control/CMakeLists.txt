cmake_minimum_required(VERSION 3.8)
project(arm_only_ee_joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_only_scan_once_cpp_node
  src/camera_only_scan_once_cpp_node.cpp
)

ament_target_dependencies(camera_only_scan_once_cpp_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

target_link_libraries(camera_only_scan_once_cpp_node
  ${OpenCV_LIBS}
)

install(TARGETS
  camera_only_scan_once_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rs_py_rgbd_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
