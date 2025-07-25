cmake_minimum_required(VERSION 3.8)
project(fernassist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AACInput.msg"
  "msg/UserIntent.msg"
  "msg/RobotAction.msg"
  "msg/LLMResponse.msg"
  "srv/ProcessAAC.srv"
  "action/ExecuteAction.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/aac_listener.py
  scripts/llm_interpreter.py
  scripts/robot_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION lib/python3/dist-packages/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 