cmake_minimum_required(VERSION 3.8)
project(rmslaunch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)
# Add executable
add_executable(cartoPose src/CartoTransferPose.cpp)
add_executable(imu_frame_transformer src/imu_frame_tf.cpp)

# Add dependencies to the executable
ament_target_dependencies(cartoPose
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  
)

ament_target_dependencies(imu_frame_transformer
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  imu_frame_transformer
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  cartoPose
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  rviz
  urdf
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()