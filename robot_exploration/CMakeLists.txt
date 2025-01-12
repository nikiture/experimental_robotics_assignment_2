cmake_minimum_required(VERSION 3.5)
project(robot_exploration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package (rclcpp REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package (std_msgs REQUIRED)
find_package (cv_bridge REQUIRED)
find_package (OpenCV REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (ros2_aruco_interfaces REQUIRED)
find_package (targets_interface REQUIRED)
find_package (nav_msgs REQUIRED)
find_package (tf2 REQUIRED)
find_package (std_msgs REQUIRED)
find_package (control_msgs REQUIRED)
#find_library (Quaternion REQUIRED)



add_executable (Robot_controller src/robot_control.cpp)
ament_target_dependencies (Robot_controller rclcpp geometry_msgs targets_interface nav_msgs std_msgs)

add_executable (simple_controller src/basic_robot_controller.cpp)
ament_target_dependencies (simple_controller rclcpp geometry_msgs)

add_executable (camera_publisher src/camera_publisher.cpp)
ament_target_dependencies (camera_publisher std_msgs rclcpp)

add_executable (Camera_controller src/camera_controller.cpp)
ament_target_dependencies (Camera_controller std_msgs rclcpp targets_interface sensor_msgs control_msgs)


install (TARGETS
	Robot_controller
	simple_controller
  camera_publisher
  Camera_controller
	DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch urdf config worlds planning_files
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
