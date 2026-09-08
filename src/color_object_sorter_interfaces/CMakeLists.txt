cmake_minimum_required(VERSION 3.8)
project(color_object_sorter_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ColorObject.msg"
  "msg/ColorObjectArray.msg"
  "msg/ConfirmedColorObject.msg"
  "msg/ConfirmedColorObjectArray.msg"
  "msg/LocalizedColorObject.msg"
  "msg/LocalizedColorObjectArray.msg"
  "msg/MappedCylinder.msg"
  "msg/CylinderField.msg"
  "msg/CylinderCandidate.msg"
  "msg/CylinderSnapshot.msg"
  "msg/ValidatedCylinder.msg"
  "msg/ValidatedCylinderArray.msg"
  "srv/SetObjectInventory.srv"
  "srv/SetSnapshotExclusions.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
