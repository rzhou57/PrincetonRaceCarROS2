cmake_minimum_required(VERSION 3.5)
project(racecar_msgs)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Specify message files
set(MSG_FILES
  "msg/ServoMsg.msg"
  "msg/OdometryArray.msg"
  "msg/SetArray.msg"
)

# Generate interfaces
# ROS 1 used add_message_files() to specify .msg files.
# ROS 2 uses rosidl_generate_interfaces(), which is more flexible and supports multiple languages (C++, Python, etc.).
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs nav_msgs
)

# Export dependencies
# In ROS 2, message packages must export their runtime dependencies.
# This ensures that other packages can find and use the generated messages.
ament_export_dependencies(rosidl_default_runtime)
ament_package() # generate CMake config files.
