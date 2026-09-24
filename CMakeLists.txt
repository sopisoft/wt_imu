cmake_minimum_required(VERSION 3.16)
project(wt_imu)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(wt_imu_protocol src/wit_protocol.cpp)
target_include_directories(wt_imu_protocol PUBLIC include)

add_executable(wt_imu_node
  src/wt_imu_node.cpp
  src/serial_port.cpp
  src/topic_publishers.cpp
  src/wit_commands.cpp)
target_include_directories(wt_imu_node PRIVATE include)
ament_target_dependencies(wt_imu_node rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(wt_imu_node wt_imu_protocol)

install(TARGETS wt_imu_node wt_imu_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_wit_protocol test/test_wit_protocol.cpp)
  target_include_directories(test_wit_protocol PRIVATE include)
  target_link_libraries(test_wit_protocol wt_imu_protocol GTest::gtest_main)
  add_test(NAME test_wit_protocol COMMAND test_wit_protocol)
  set_tests_properties(test_wit_protocol PROPERTIES LABELS gtest)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
