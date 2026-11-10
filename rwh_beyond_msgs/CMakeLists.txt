cmake_minimum_required(VERSION 3.16)
project(rwh_beyond_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TcpPose.msg"
  "action/RunDemo.action"
  "srv/GetStatus.srv"
  DEPENDENCIES geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
