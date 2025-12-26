cmake_minimum_required(VERSION 3.8)
project(rplidar_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# rplidar_listener
add_executable(rplidar_listener src/rplidar_listener.cpp)
ament_target_dependencies(
  rplidar_listener
  "rclcpp"
  "sensor_msgs"
)
install(TARGETS rplidar_listener
  DESTINATION lib/${PROJECT_NAME})

# rplidar_info
add_executable(rplidar_info src/rplidar_info.cpp)
ament_target_dependencies(
  rplidar_info
  "rclcpp"
  "sensor_msgs"
)
install(TARGETS rplidar_info
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
