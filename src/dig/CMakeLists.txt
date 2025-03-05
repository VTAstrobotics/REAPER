cmake_minimum_required(VERSION 3.8)
project(dig)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(phoenix6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(action_interfaces REQUIRED)
find_package(state_messages REQUIRED)
find_package(state_messages_utils REQUIRED)

find_package(sparkcan REQUIRED)
find_package(std_msgs REQUIRED)
find_package(state_messages REQUIRED)
find_package(state_messages_utils REQUIRED)

find_package(unique_identifier_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
message(STATUS "Include directories: ${CMAKE_INCLUDE_PATH}")

add_library(Dig_action SHARED src/dig_server.cpp)
target_include_directories(Dig_action PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(Dig_action PRIVATE
"DIG_ACTION_CPP_BUILDING_DLL")

ament_target_dependencies(Dig_action
  "action_interfaces"
  "state_messages"
  "state_messages_utils"
  "sparkcan"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "phoenix6"
  "state_messages"
  "state_messages_utils"
  )
rclcpp_components_register_node(Dig_action PLUGIN "dig_server::DigActionServer" EXECUTABLE DigActionServer)

add_definitions(-DUNIT_LIB_DISABLE_FMT -DUNIT_LIB_ENABLE_IOSTREAM)
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

install(TARGETS
  Dig_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_package()