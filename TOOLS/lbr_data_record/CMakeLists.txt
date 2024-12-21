cmake_minimum_required(VERSION 3.8)
project(lbr_data_record)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)


add_executable(lbr_data_record src/lbr_data_record.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  control_msgs
)

include_directories(../../PARAM)  # include param header file

target_include_directories(lbr_data_record PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(lbr_data_record PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(
  DIRECTORY script
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS lbr_data_record
  DESTINATION lib/${PROJECT_NAME})

ament_package()
