cmake_minimum_required(VERSION 3.20)
project(easynav_vff_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_core
  pluginlib
  visualization_msgs
)

add_library(vff_controller SHARED
  src/easynav_vff_controller/VffController.cpp
)
target_include_directories(vff_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vff_controller
  easynav_core
  pluginlib
  visualization_msgs
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  vff_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  vff_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)
# Register the control plugins
pluginlib_export_plugin_description_file(easynav_core easynav_vff_controller_plugins.xml)
ament_export_dependencies(${dependencies})
ament_package()
