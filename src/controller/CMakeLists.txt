cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(longitudinal_controller_node src/controller.cpp)
ament_target_dependencies(longitudinal_controller_node rclcpp std_msgs)
target_compile_features(longitudinal_controller_node PUBLIC cxx_std_17)

install(TARGETS
  longitudinal_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()