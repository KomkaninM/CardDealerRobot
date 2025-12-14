cmake_minimum_required(VERSION 3.8)
project(card_dealer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Player.msg"
  "msg/RoomConfig.msg"
)

# 3. Export dependencies
# This ensures that when other packages depend on this one, 
# they get the correct runtime libraries.
ament_export_dependencies(rosidl_default_runtime)

# 4. Package hook
ament_package()
