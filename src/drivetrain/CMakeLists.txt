<<<<<<< HEAD
cmake_minimum_required(VERSION 3.8)
project(drivetrain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(phoenix6 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Drive.action"
  DEPENDENCIES action_msgs geometry_msgs
)

# Include directories
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(Drive_action_server SHARED src/drive_server.cpp)
target_include_directories(Drive_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(Drive_action_server PRIVATE "DRIVE_ACTION_CPP_BUILDING_DLL")


ament_target_dependencies(Drive_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "action_interfaces"
  "phoenix6"
)

rclcpp_components_register_node(Drive_action_server PLUGIN "drive_server::DriveActionServer" EXECUTABLE DriveActionServer)

install(TARGETS
  Drive_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
=======
cmake_minimum_required(VERSION 3.8)
project(drivetrain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
>>>>>>> 60c14ac (Brett's commitment <3)
