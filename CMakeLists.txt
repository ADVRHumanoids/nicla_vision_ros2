cmake_minimum_required(VERSION 3.10.2)
project(nicla_vision_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Commons
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(xacro REQUIRED) #to launch nicla_sim
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
#To view images
find_package(image_transport_plugins REQUIRED)
#For messages
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(OpenCV)


###########
## Messages ##
###########
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  "msg/AudioDataStamped.msg"
  "msg/AudioInfo.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

###########
## Linters ##
###########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/nicla_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()