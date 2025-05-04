cmake_minimum_required(VERSION 3.8)
project(odrive_diff_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(diff_drive src/odrive_diff_drive.cpp)
ament_target_dependencies(diff_drive rclcpp geometry_msgs std_msgs)

install(TARGETS diff_drive 
        DESTINATION lib/${PROJECT_NAME})

ament_package()
