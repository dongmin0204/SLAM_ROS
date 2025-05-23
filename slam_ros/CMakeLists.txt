cmake_minimum_required(VERSION 3.8)
project(slam_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_eigen REQUIRED)

# Generate action messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveXY.action"
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add the movement recorder executable
add_executable(movement_recorder src/movement_recorder.cpp)
ament_target_dependencies(movement_recorder
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# Add the movement controller executable
add_executable(movement_controller 
  src/movement_controller.cpp
  src/quintic_trajectory.cpp
  src/kinematic_model.cpp
  src/mpc_solver.cpp
)

ament_target_dependencies(movement_controller
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(movement_controller
  Eigen3::Eigen
  osqp_eigen
)

rosidl_target_interfaces(movement_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executables
install(TARGETS
  movement_recorder
  movement_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package() 