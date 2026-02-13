cmake_minimum_required(VERSION 3.8)
project(chudovishe_serial_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(serial_node src/chudovishe_serial_test.cpp)

ament_target_dependencies(serial_node
  rclcpp
  std_msgs
  serial
)

# На случай, если serial собран как catkin и не экспортирует targets как ament:
if(DEFINED serial_INCLUDE_DIRS)
  target_include_directories(serial_node PRIVATE ${serial_INCLUDE_DIRS})
endif()

install(TARGETS serial_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
