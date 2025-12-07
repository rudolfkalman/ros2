cmake_minimum_required(VERSION 3.8)
project(test_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Build Node
add_executable(string_publisher src/test_cpp.cpp)
target_link_libraries(string_publisher Eigen3::Eigen)

ament_target_dependencies(string_publisher rclcpp std_msgs)

# Install
install(TARGETS
  string_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Test
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  set(ament_cmake_copyright_FOUND TRUE)
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()

