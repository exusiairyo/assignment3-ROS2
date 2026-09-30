cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(camera_node
  
  src/main.cpp
  src/camera_node.cpp
)
target_include_directories(camera_node PRIVATE include /opt/MVS/include)
  target_link_directories(camera_node PRIVATE /opt/MVS/lib/64)
  target_link_libraries(camera_node MvCameraControl)
target_compile_features(camera_node PRIVATE cxx_std_17)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

# TODO(student): Declare and link the dependencies used by your implementation,
# including the vendor SDK. The starter does not discover or link MVS.
ament_target_dependencies(camera_node rclcpp sensor_msgs)
install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

