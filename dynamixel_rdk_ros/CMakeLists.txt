# DYNAMIXEL_RDK_ROS_CMAKELISTS.txt

cmake_minimum_required(VERSION 3.8)
project(dynamixel_rdk_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp_lifecycle REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_rdk_msgs REQUIRED)

include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
)

# Build dynamixel_rdk_node
add_executable(dynamixel_rdk_node src/dynamixel_rdk_node.cpp src/dynamixel_ctrl.cpp src/dynamixel.cpp)
target_link_libraries(dynamixel_rdk_node ${dynamixel_sdk_LIBRARIES})
ament_target_dependencies(dynamixel_rdk_node rclcpp rclcpp_lifecycle dynamixel_sdk dynamixel_rdk_msgs)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  dynamixel_rdk_node
  DESTINATION lib/${PROJECT_NAME})

# Register the package
ament_package()
