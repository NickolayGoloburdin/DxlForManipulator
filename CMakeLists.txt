cmake_minimum_required(VERSION 3.8)
project(arm_packet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

include_directories(include)
add_executable(arm_controll src/arm_controll.cpp)
ament_target_dependencies(arm_controll rclcpp std_msgs sensor_msgs dynamixel_sdk dynamixel_workbench_toolbox)
install(TARGETS
  arm_controll
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(dynamixel_sdk)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_package()
