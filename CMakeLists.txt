cmake_minimum_required(VERSION 3.20)
project(easynav_outdoor_testcase)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robots_params/
  DESTINATION share/${PROJECT_NAME}/robots_params
)

ament_package()

