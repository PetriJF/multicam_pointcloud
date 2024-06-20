cmake_minimum_required(VERSION 3.8)
project(multicam_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/includes
  ${OpenCV_INCLUDE_DIRS}
  /home/edward/librealsense/include
)

# Link directories
link_directories(/home/edward/librealsense/lib)

# Add executable
add_executable(realsense_multicam_node src/realsense_multicam_node.cpp)

# Link the libraries
target_link_libraries(realsense_multicam_node
  ${OpenCV_LIBS}
  realsense2
)

# Dependencies for ament
ament_target_dependencies(realsense_multicam_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install executable
install(TARGETS
  realsense_multicam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
