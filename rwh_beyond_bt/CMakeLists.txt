cmake_minimum_required(VERSION 3.16)
project(rwh_beyond_bt)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/register_nodes.cpp
  src/action/move_home.cpp
  src/action/move_tcp_up.cpp
  src/action/close_gripper.cpp
  src/action/open_gripper.cpp
  src/action/set_gripper.cpp
  src/action/move_tcp.cpp)
target_include_directories(${PROJECT_NAME}_nodes PRIVATE include)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(${PROJECT_NAME}_nodes
  behaviortree_cpp control_msgs geometry_msgs moveit_msgs nav2_behavior_tree rclcpp
  rclcpp_action shape_msgs tf2_geometry_msgs tf2_ros trajectory_msgs)

install(TARGETS ${PROJECT_NAME}_nodes LIBRARY DESTINATION lib)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
add_executable(generate_tree_nodes_xml src/generate_tree_nodes_xml.cpp)
target_compile_features(generate_tree_nodes_xml PRIVATE cxx_std_17)
ament_target_dependencies(generate_tree_nodes_xml behaviortree_cpp)
install(TARGETS generate_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})
install(FILES rwh_beyond_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_custom_actions test/test_custom_actions.cpp)
  ament_target_dependencies(test_custom_actions
    behaviortree_cpp control_msgs geometry_msgs moveit_msgs rclcpp rclcpp_action tf2_ros)
  target_compile_features(test_custom_actions PRIVATE cxx_std_17)
  target_compile_definitions(test_custom_actions PRIVATE
    BT_PLUGIN_PATH="$<TARGET_FILE:${PROJECT_NAME}_nodes>")
  add_dependencies(test_custom_actions ${PROJECT_NAME}_nodes)
  set_tests_properties(test_custom_actions PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=87")
endif()

ament_package()
