cmake_minimum_required(VERSION 3.14.4)
project (norbit_driver)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(norbit_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)


# norbit library

add_library(norbit SHARED
  src/bathymetric_data.cpp
  src/conversions.cpp
  src/message.cpp
  src/norbit_connection.cpp
  src/water_column_data.cpp
)

target_include_directories(norbit
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

target_link_libraries(norbit 
  rclcpp_lifecycle::rclcpp_lifecycle
  ${marine_acoustic_msgs_TARGETS}
  ${norbit_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)

ament_target_dependencies(norbit
  pcl_ros
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS norbit
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# norbit node

add_executable(norbit_node
  nodes/norbit_node.cpp
)

target_link_libraries(norbit_node
  norbit
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

install (TARGETS norbit_node
  DESTINATION lib/${PROJECT_NAME}
)

# package

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  marine_acoustic_msgs
  norbit_interfaces
  pcl_ros
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

ament_package()

