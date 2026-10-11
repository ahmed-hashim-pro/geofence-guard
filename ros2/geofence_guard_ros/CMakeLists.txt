cmake_minimum_required(VERSION 3.16)
project(geofence_guard_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geofence_guard_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(guard_node SHARED src/guard_node.cpp)
target_include_directories(guard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(guard_node PUBLIC
  geofence_guard::geofence_guard_core
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS})

add_executable(geofence_guard_node src/main.cpp)
target_link_libraries(geofence_guard_node guard_node)

install(TARGETS guard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS geofence_guard_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_guard_node test/test_guard_node.cpp TIMEOUT 60)
  target_link_libraries(test_guard_node guard_node)
  target_compile_definitions(test_guard_node PRIVATE
    TEST_ENVELOPE="${CMAKE_CURRENT_SOURCE_DIR}/config/field.fence")
endif()

ament_package()
