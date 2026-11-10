cmake_minimum_required(VERSION 3.16)
project(rwh_beyond_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TcpPose.msg"
  "msg/GraspCandidate.msg"
  "action/InferGrasps.action"
  "action/RunGraspStep.action"
  "action/RunDemo.action"
  "srv/GetStatus.srv"
  DEPENDENCIES geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
