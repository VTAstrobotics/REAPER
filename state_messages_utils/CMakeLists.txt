cmake_minimum_required(VERSION 3.8)
project(state_messages_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(phoenix6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(state_messages REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(sparkcan REQUIRED)
find_package(std_msgs REQUIRED)
set(HEADER_FILES include/state_messages_utils/motor_to_msg.hpp)
add_library(state_messages_utils SHARED src/motor_to_msg.cpp ${HEADER_FILES})

ament_target_dependencies(state_messages_utils
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "state_messages"
  "action_interfaces"
  "sparkcan"
  "std_msgs"
  "rosidl_default_runtime"
  "phoenix6"
  )

target_include_directories(
  state_messages_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

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

install(
  DIRECTORY include/state_messages_utils
  DESTINATION include
)

install(
  TARGETS state_messages_utils
  EXPORT state_messages_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/state_messages_utils
)

ament_export_include_directories(include)

ament_package()
