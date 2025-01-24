cmake_minimum_required(VERSION 3.5)
project(openscenario_visualization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(traffic_simulator REQUIRED)
find_package(openscenario_msgs REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(openscenario_visualization_component SHARED
  src/openscenario_visualization_component.cpp
)
ament_target_dependencies(openscenario_visualization_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  traffic_simulator
  visualization_msgs
  openscenario_msgs
  quaternion_operation
)
add_executable(openscenario_visualization_node
  src/openscenario_visualization_node.cpp
)
target_link_libraries(openscenario_visualization_node openscenario_visualization_component)
ament_target_dependencies(openscenario_visualization_node
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  traffic_simulator
  visualization_msgs
  openscenario_msgs
  quaternion_operation
)

install(TARGETS
  openscenario_visualization_node
  DESTINATION lib/openscenario_visualization
)
install(TARGETS
  openscenario_visualization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(openscenario_visualization_component)
rclcpp_components_register_nodes(openscenario_visualization_component
  "openscenario_visualization::OpenscenarioVisualizationComponent")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
