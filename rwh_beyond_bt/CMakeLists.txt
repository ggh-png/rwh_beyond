cmake_minimum_required(VERSION 3.16)
project(rwh_beyond_bt)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration segmentation)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rwh_beyond_msgs REQUIRED)

set(dependencies
  behaviortree_cpp geometry_msgs moveit_msgs nav2_behavior_tree rclcpp
  rclcpp_action rwh_beyond_msgs sensor_msgs std_srvs control_msgs trajectory_msgs tf2_ros urdf shape_msgs rcl_interfaces visualization_msgs)

add_library(${PROJECT_NAME}_common SHARED
  src/diagnostics.cpp)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_common PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_common PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_common ${dependencies})

add_library(rwh_beyond_joint_pose_store SHARED src/joint_pose_store.cpp)
target_include_directories(rwh_beyond_joint_pose_store PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rwh_beyond_joint_pose_store PUBLIC cxx_std_17)
target_link_libraries(rwh_beyond_joint_pose_store yaml-cpp)
ament_target_dependencies(rwh_beyond_joint_pose_store ament_index_cpp yaml_cpp_vendor)

set(joint_poses_source "${CMAKE_CURRENT_SOURCE_DIR}/../config/joint_poses.yaml")
set(joint_poses_snapshot "${CMAKE_CURRENT_BINARY_DIR}/joint_poses.yaml")
target_compile_definitions(rwh_beyond_joint_pose_store PRIVATE
  RWH_JOINT_POSES_SOURCE="${joint_poses_source}")
add_custom_target(snapshot_joint_poses ALL
  COMMAND "${CMAKE_COMMAND}" "-DSOURCE=${joint_poses_source}" "-DSNAPSHOT=${joint_poses_snapshot}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/snapshot_joint_poses.cmake"
  BYPRODUCTS "${joint_poses_snapshot}"
  VERBATIM)
install(FILES "${joint_poses_snapshot}" DESTINATION share/${PROJECT_NAME}/config)

add_executable(capture_joint_pose src/capture_joint_pose.cpp)
target_link_libraries(capture_joint_pose rwh_beyond_joint_pose_store)
ament_target_dependencies(capture_joint_pose rclcpp sensor_msgs)
target_compile_features(capture_joint_pose PRIVATE cxx_std_17)
install(TARGETS capture_joint_pose DESTINATION lib/${PROJECT_NAME})

set(plugin_libs)
function(add_bt_action_plugin name)
  set(target rwh_beyond_${name}_action_bt_node)
  add_library(${target} SHARED plugins/action/${name}_action.cpp)
  target_link_libraries(${target} ${PROJECT_NAME}_common)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${target} ${dependencies})
  list(APPEND plugin_libs ${target})
  set(plugin_libs ${plugin_libs} PARENT_SCOPE)
endfunction()

add_bt_action_plugin(clear_octomap)

add_library(rwh_beyond_perception_bt_nodes SHARED plugins/action/perception_actions.cpp)
target_include_directories(rwh_beyond_perception_bt_nodes SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rwh_beyond_perception_bt_nodes PUBLIC ${PROJECT_NAME}_common
  PRIVATE pcl_segmentation)
target_compile_features(rwh_beyond_perception_bt_nodes PUBLIC cxx_std_17)
target_compile_definitions(rwh_beyond_perception_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(rwh_beyond_perception_bt_nodes PUBLIC ${dependencies})
list(APPEND plugin_libs rwh_beyond_perception_bt_nodes)

add_library(rwh_beyond_motion_bt_nodes SHARED
  plugins/action/motion_actions.cpp
  plugins/action/motion_action.cpp
  plugins/action/check_grasp_ready_action.cpp
  plugins/action/select_feasible_grasp_action.cpp
  plugins/action/move_to_pregrasp_action.cpp
  plugins/action/approach_grasp_action.cpp
  plugins/action/attach_object_action.cpp
  plugins/action/lift_object_action.cpp
  plugins/action/move_home_action.cpp
  plugins/action/move_observation_action.cpp
  plugins/action/move_place_action.cpp
  plugins/action/open_gripper_action.cpp
  plugins/action/release_object_action.cpp
  src/motion_context.cpp
  src/multiview.cpp
  plugins/action/multiview_actions.cpp
  plugins/action/joint_pose_actions.cpp)
target_link_libraries(rwh_beyond_motion_bt_nodes PUBLIC rwh_beyond_joint_pose_store ${PROJECT_NAME}_common rwh_beyond_perception_bt_nodes Eigen3::Eigen)
target_include_directories(rwh_beyond_motion_bt_nodes SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rwh_beyond_motion_bt_nodes PRIVATE ${PCL_LIBRARIES})
target_compile_features(rwh_beyond_motion_bt_nodes PUBLIC cxx_std_17)
target_compile_definitions(rwh_beyond_motion_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(rwh_beyond_motion_bt_nodes PUBLIC ${dependencies})
list(APPEND plugin_libs rwh_beyond_motion_bt_nodes)

# Compatibility loader; individual plugins can also be loaded directly by Nav2.
add_library(${PROJECT_NAME}_nodes SHARED plugins/register_nodes.cpp)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(${PROJECT_NAME}_nodes behaviortree_cpp)
add_dependencies(${PROJECT_NAME}_nodes ${plugin_libs})

install(TARGETS rwh_beyond_joint_pose_store ${PROJECT_NAME}_common ${PROJECT_NAME}_nodes ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
add_executable(generate_tree_nodes_xml src/generate_tree_nodes_xml.cpp)
target_compile_features(generate_tree_nodes_xml PRIVATE cxx_std_17)
ament_target_dependencies(generate_tree_nodes_xml behaviortree_cpp)
install(TARGETS generate_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})
install(FILES node_models/rwh_beyond_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cpplint(include plugins src test)
  ament_uncrustify(include plugins src test)
  ament_add_gtest(test_bt_actions test/test_bt_actions.cpp)
  ament_target_dependencies(test_bt_actions
    behaviortree_cpp geometry_msgs rclcpp rclcpp_action rwh_beyond_msgs sensor_msgs)
  target_link_libraries(test_bt_actions rwh_beyond_perception_bt_nodes rwh_beyond_motion_bt_nodes)
  ament_target_dependencies(test_bt_actions ${dependencies})
  target_compile_features(test_bt_actions PRIVATE cxx_std_17)
  target_compile_definitions(test_bt_actions PRIVATE
    BT_PLUGIN_PATH="$<TARGET_FILE:${PROJECT_NAME}_nodes>"
    BT_PLUGIN_DIR="$<TARGET_FILE_DIR:${PROJECT_NAME}_nodes>"
    BT_TREE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  add_dependencies(test_bt_actions ${PROJECT_NAME}_nodes)
  set_tests_properties(test_bt_actions PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=87")
  ament_add_gtest(test_joint_pose_store test/test_joint_pose_store.cpp)
  target_link_libraries(test_joint_pose_store rwh_beyond_joint_pose_store)
  target_compile_features(test_joint_pose_store PRIVATE cxx_std_17)
  ament_add_gtest(test_multiview test/test_multiview.cpp)
  ament_target_dependencies(test_multiview ${dependencies})
  target_link_libraries(test_multiview rwh_beyond_motion_bt_nodes Eigen3::Eigen)
  target_compile_features(test_multiview PRIVATE cxx_std_17)
  ament_add_gtest(test_motion test/test_motion.cpp)
  ament_target_dependencies(test_motion ${dependencies})
  target_link_libraries(test_motion rwh_beyond_motion_bt_nodes Eigen3::Eigen)
  target_compile_features(test_motion PRIVATE cxx_std_17)
  target_compile_definitions(test_motion PRIVATE BT_PLUGIN_PATH="$<TARGET_FILE:${PROJECT_NAME}_nodes>")
  add_dependencies(test_motion ${PROJECT_NAME}_nodes)
  set_tests_properties(test_motion PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=88")
  ament_add_gtest(test_clear_octomap test/test_clear_octomap.cpp)
  target_link_libraries(test_clear_octomap ${PROJECT_NAME}_common)
  ament_target_dependencies(test_clear_octomap
    behaviortree_cpp moveit_msgs rclcpp rwh_beyond_msgs sensor_msgs std_srvs)
  target_compile_features(test_clear_octomap PRIVATE cxx_std_17)
  target_compile_definitions(test_clear_octomap PRIVATE BT_PLUGIN_PATH="$<TARGET_FILE:${PROJECT_NAME}_nodes>")
  add_dependencies(test_clear_octomap ${PROJECT_NAME}_nodes)
  set_tests_properties(test_clear_octomap PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=90")
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} Eigen3 ament_index_cpp yaml_cpp_vendor yaml-cpp)
ament_package()
