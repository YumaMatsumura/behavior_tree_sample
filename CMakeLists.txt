cmake_minimum_required(VERSION 3.8)
project(behavior_tree_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  behaviortree_cpp_v3
)

add_executable(bt_engine
  src/main.cpp
  src/bt_engine.cpp
)
ament_target_dependencies(bt_engine ${dependencies})

add_library(hello_world_action_bt_node SHARED plugins/action/hello_world_action.cpp)
list(APPEND plugin_libs hello_world_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bt_engine
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})

ament_package()
