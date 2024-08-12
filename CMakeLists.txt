cmake_minimum_required(VERSION 3.8)
project(multicam_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED)  # Ensure librealsense2 is found
find_package(farmbot_interfaces REQUIRED)    # Ensure std_srvs is found

# Define project directories
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/includes)

# Add executable
# add_executable(realsense_multicam_node src/realsense_multicam_node.cpp)
add_executable(toggle_client_test src/toggle_client_test.cpp)

# Include directories
include_directories(
  ${PROJECT_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  # ${realsense2_INCLUDE_DIRS}
)

# Link the libraries
# target_link_libraries(realsense_multicam_node
  # ${OpenCV_LIBS}
  # ${realsense2_LIBRARY}
# )

# Dependencies for ament
# ament_target_dependencies(realsense_multicam_node
#  rclcpp
#  sensor_msgs
#  cv_bridge
#  farmbot_interfaces
# )

ament_target_dependencies(toggle_client_test
  rclcpp
  farmbot_interfaces
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable
install(TARGETS
  # realsense_multicam_node
  # toggle_client_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/data_collection_node.py
  scripts/mcpc_node.py
  scripts/luxonis_multicam_node.py
  scripts/luxonis_multicam_subscriber_test.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
