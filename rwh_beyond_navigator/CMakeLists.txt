cmake_minimum_required(VERSION 3.16)
project(rwh_beyond_navigator)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rwh_beyond_msgs REQUIRED)
find_package(rwh_beyond_bt REQUIRED)

add_library(${PROJECT_NAME} SHARED src/demo_navigator.cpp src/xml_tree_filter.cpp)
target_link_libraries(${PROJECT_NAME} tinyxml2::tinyxml2 rwh_beyond_bt::rwh_beyond_bt_common)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp nav2_behavior_tree nav2_core nav2_util pluginlib rclcpp rclcpp_action
  rwh_beyond_msgs rwh_beyond_bt)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(FILES navigator_plugins.xml DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_xml_filter test/test_xml_filter.cpp)
  target_include_directories(test_xml_filter PRIVATE src)
  target_link_libraries(test_xml_filter ${PROJECT_NAME} tinyxml2::tinyxml2)
  target_compile_features(test_xml_filter PRIVATE cxx_std_17)
  ament_add_pytest_test(error_reporting test/test_error_reporting.py
    TIMEOUT 90 ENV ROS_DOMAIN_ID=89 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(groot_monitoring test/test_groot_monitoring.py
    TIMEOUT 90 ENV ROS_DOMAIN_ID=90 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
