cmake_minimum_required(VERSION 3.14)
project(inspire_hand_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingerTactile.msg"
  "msg/InspireHandState.msg"
  "msg/InspireHandControl.msg"
  "msg/InspireHandTouch.msg"
  "msg/GraspStatus.msg"
  "srv/Grasp.srv"
  "srv/Release.srv"
  "srv/SetFingerPosition.srv"
  "srv/GetHandState.srv"
  "action/GraspObject.action"
  DEPENDENCIES std_msgs builtin_interfaces action_msgs
)

# Install Python executable nodes
install(PROGRAMS
  inspire_hand_ros2/inspire_hand_node.py
  inspire_hand_ros2/visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python support modules next to the executable
install(FILES
  inspire_hand_ros2/inspire_hand_driver.py
  inspire_hand_ros2/tactile_processor.py
  inspire_hand_ros2/grasp_controller.py
  inspire_hand_ros2/viz_data.py
  inspire_hand_ros2/viz_qt.py
  inspire_hand_ros2/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
