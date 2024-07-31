cmake_minimum_required(VERSION 3.8)
project(fsai_self_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

include_directories(
  include  
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
)

ament_auto_add_library(steer_component SHARED
  src/steer_component.cpp 
)
target_link_libraries(steer_component
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_nodes(steer_component
  "self_drive::Steer"
)
ament_auto_add_executable(steer_node
  src/steer_node.cpp
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
