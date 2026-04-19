cmake_minimum_required(VERSION 3.16)
project(ohm_ros2 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS 2 dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

# OHM (CUDA backend). ohm-config.cmake populates OHM_GPU_LIBRARY (ohm::ohmcuda)
# and OHM_GPUTIL_LIBRARY (ohm::gputilcuda) when COMPONENTS CUDA is requested.
# glm is header-only; libglm-dev (Jammy) ships a broken glmConfig-version.cmake,
# so we skip find_package(glm) and consume its headers via OHM_INCLUDE_DIRS
# (OHM's own targets already depend on glm).
find_package(ohm REQUIRED COMPONENTS CUDA)

add_executable(ohm_live_node src/ohm_live_node.cpp)

target_include_directories(ohm_live_node PRIVATE
  ${OHM_INCLUDE_DIRS}
)

# ohm::ohm gives us OccupancyMap, ohm::ohmcuda gives us GpuMap/GpuNdtMap.
target_link_libraries(ohm_live_node
  ${OHM_GPU_LIBRARY}
  ${OHM_GPUTIL_LIBRARY}
  ohm::ohm
)

ament_target_dependencies(ohm_live_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  message_filters
)

install(TARGETS ohm_live_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
