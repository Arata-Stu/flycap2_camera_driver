cmake_minimum_required(VERSION 3.8)
project(flycap2_camera_driver)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(/usr/include/flycapture)
link_directories(/usr/lib)

add_executable(
  grasshopper3
  src/grasshopper3.cpp
)

ament_target_dependencies(grasshopper3 rclcpp sensor_msgs)
TARGET_LINK_LIBRARIES(grasshopper3 /usr/lib/libflycapture.so -lpthread -lm)

install(TARGETS
  grasshopper3
  DESTINATION lib/${PROJECT_NAME})

ament_package()
