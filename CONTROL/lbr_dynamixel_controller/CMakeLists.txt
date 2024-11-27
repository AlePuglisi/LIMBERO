cmake_minimum_required(VERSION 3.8)
project(lbr_dynamixel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(lbr_msgs REQUIRED)

include_directories(include)

add_executable(lbr_dynamixel_controller src/lbr_dynamixel_controller.cpp)
ament_target_dependencies(lbr_dynamixel_controller
  rclcpp
  std_msgs
  lbr_msgs
  sensor_msgs
  dynamixel_sdk)
target_include_directories(lbr_dynamixel_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lbr_dynamixel_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lbr_dynamixel_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

include_directories(../../PARAM)  # include param header file

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
