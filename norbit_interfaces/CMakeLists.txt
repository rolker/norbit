cmake_minimum_required(VERSION 3.8)
project(norbit_interfaces)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/BathymetricHeader.msg
  msg/Bathymetric.msg
  msg/BathymetricPoint.msg
  msg/BathymetricStamped.msg
  msg/CmdResp.msg
  msg/CommonHeader.msg
  msg/WaterColumnHeader.msg
  msg/WaterColumn.msg
  msg/WaterColumnStamped.msg
  srv/NorbitCmd.srv
  srv/SetDirection.srv
  srv/SetFlip.srv
  srv/SetGateMode.srv
  srv/SetGateTilt.srv
  srv/SetMfDecimation.srv
  srv/SetMode.srv
  srv/SetOpeningAngle.srv
  srv/SetPower.srv
  srv/SetRange.srv
  srv/SetResolution.srv
  srv/SetSidescanMode.srv
  srv/SetTxBeamNumber.srv
  srv/SetTxDirection.srv
  srv/SetTxOpeningAngle.srv
  DEPENDENCIES
  builtin_interfaces
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

