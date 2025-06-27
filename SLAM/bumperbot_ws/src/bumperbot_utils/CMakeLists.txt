cmake_minimum_required(VERSION 3.8)
project(bumperbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(trajectory_plotter src/trajectory_plotter.cpp)
ament_target_dependencies(trajectory_plotter rclcpp nav_msgs geometry_msgs)

install(PROGRAMS
  ${PROJECT_NAME}/trajectory_plotter.py
  ${PROJECT_NAME}/safety_stop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  trajectory_plotter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
