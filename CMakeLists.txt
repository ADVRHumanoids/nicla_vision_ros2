cmake_minimum_required(VERSION 3.10.2)
project(nicla_vision_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport_plugins REQUIRED)
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

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY config/rviz
  DESTINATION share/${PROJECT_NAME}/config/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.xacro"
)
install(DIRECTORY urdf/meshes
  DESTINATION share/${PROJECT_NAME}/urdf/meshes
  FILES_MATCHING PATTERN "*.dae"
)
install(DIRECTORY urdf/meshes
  DESTINATION share/${PROJECT_NAME}/urdf/meshes
  FILES_MATCHING PATTERN "*.stl"
)

ament_package()