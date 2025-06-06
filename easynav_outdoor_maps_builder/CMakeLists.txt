cmake_minimum_required(VERSION 3.20)
project(easynav_outdoor_maps_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(easynav_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  geometry_msgs
  easynav_common
  grid_map_ros
  grid_map_msgs
)

include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/easynav_outdoor_maps_builder/OutdoorMapsBuilderNode.cpp
  src/easynav_outdoor_maps_builder/types/PointcloudMapsBuilder.cpp
  src/easynav_outdoor_maps_builder/types/GridMapMapsBuilder.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Executable
add_executable(outdoor_maps_builder_main src/outdoor_maps_builder_main.cpp)
ament_target_dependencies(outdoor_maps_builder_main ${dependencies})
target_link_libraries(outdoor_maps_builder_main ${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  outdoor_maps_builder_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
