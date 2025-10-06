cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  number_publisher
  number_counter
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
    # Add the test executable
  ament_add_gtest(test_robot_news_station test/test_robot_news_station.cpp)
  
  if(TARGET test_robot_news_station)
    target_link_libraries(test_robot_news_station ${rclcpp_LIBRARIES})
    ament_target_dependencies(test_robot_news_station 
      rclcpp 
      example_interfaces
    )
  endif()
  
  # Add more tests as needed
  # ament_add_gtest(test_number_publisher test/test_number_publisher.cpp)
  # if(TARGET test_number_publisher)
  #   target_link_libraries(test_number_publisher ${rclcpp_LIBRARIES})
  #   ament_target_dependencies(test_number_publisher rclcpp example_interfaces)
  # endif()
  
  # Linting tests
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
