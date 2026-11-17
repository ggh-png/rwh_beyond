cmake_minimum_required(VERSION 3.16)
project(rwh_beyond_bt)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rwh_beyond_msgs REQUIRED)

set(dependencies
  behaviortree_cpp geometry_msgs moveit_msgs nav2_behavior_tree rclcpp
  rclcpp_action rwh_beyond_msgs sensor_msgs std_srvs)

add_library(${PROJECT_NAME}_common SHARED
  src/bt_scenario_action_node.cpp
  src/bt_grasp_action_node.cpp
  src/diagnostics.cpp)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_common PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_common PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_common ${dependencies})

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
add_bt_action_plugin(move_home)
add_bt_action_plugin(move_observation)
add_bt_action_plugin(move_place)
add_bt_action_plugin(open_gripper)
add_bt_action_plugin(release_object)
add_bt_action_plugin(check_grasp_ready)
add_bt_action_plugin(select_feasible_grasp)
add_bt_action_plugin(move_to_pregrasp)
add_bt_action_plugin(approach_grasp)
add_bt_action_plugin(attach_object)
add_bt_action_plugin(lift_object)

add_library(rwh_beyond_perception_bt_nodes SHARED plugins/action/perception_actions.cpp)
target_link_libraries(rwh_beyond_perception_bt_nodes ${PROJECT_NAME}_common)
target_compile_features(rwh_beyond_perception_bt_nodes PUBLIC cxx_std_17)
target_compile_definitions(rwh_beyond_perception_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(rwh_beyond_perception_bt_nodes ${dependencies})
list(APPEND plugin_libs rwh_beyond_perception_bt_nodes)

# Compatibility loader; individual plugins can also be loaded directly by Nav2.
add_library(${PROJECT_NAME}_nodes SHARED src/register_nodes.cpp)
target_compile_features(${PROJECT_NAME}_nodes PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(${PROJECT_NAME}_nodes behaviortree_cpp)
add_dependencies(${PROJECT_NAME}_nodes ${plugin_libs})

install(TARGETS ${PROJECT_NAME}_common ${PROJECT_NAME}_nodes ${plugin_libs}
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
  target_link_libraries(test_bt_actions rwh_beyond_perception_bt_nodes)
  target_compile_features(test_bt_actions PRIVATE cxx_std_17)
  target_compile_definitions(test_bt_actions PRIVATE
    BT_PLUGIN_PATH="$<TARGET_FILE:${PROJECT_NAME}_nodes>"
    BT_PLUGIN_DIR="$<TARGET_FILE_DIR:${PROJECT_NAME}_nodes>")
  add_dependencies(test_bt_actions ${PROJECT_NAME}_nodes)
  set_tests_properties(test_bt_actions PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=87")
  ament_add_gtest(test_clear_octomap test/test_clear_octomap.cpp)
  ament_target_dependencies(test_clear_octomap
    behaviortree_cpp moveit_msgs rclcpp rwh_beyond_msgs sensor_msgs std_srvs)
  target_compile_features(test_clear_octomap PRIVATE cxx_std_17)
  target_compile_definitions(test_clear_octomap PRIVATE BT_PLUGIN_PATH="$<TARGET_FILE:${PROJECT_NAME}_nodes>")
  add_dependencies(test_clear_octomap ${PROJECT_NAME}_nodes)
  set_tests_properties(test_clear_octomap PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=90")
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
