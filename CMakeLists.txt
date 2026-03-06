cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(gz_dvs_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Gazebo Harmonic
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# ---- Messages ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DvsEvent.msg"
  "msg/DvsEventArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# ---- Plugin Library ----
add_library(gz-dvs-system SHARED
  src/DvsSystem.cc
)

target_include_directories(gz-dvs-system PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(gz-dvs-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-rendering8::gz-rendering8
  gz-common5::gz-common5
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(gz-dvs-system PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gz-dvs-system PRIVATE ${cpp_typesupport_target})

# ---- Install ----
install(TARGETS gz-dvs-system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
