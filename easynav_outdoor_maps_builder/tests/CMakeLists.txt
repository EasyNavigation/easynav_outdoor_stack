ament_add_gtest(outdoor_maps_builder_tests outdoor_maps_builder_tests.cpp)
target_include_directories(outdoor_maps_builder_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(outdoor_maps_builder_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(outdoor_maps_builder_tests ${dependencies})
