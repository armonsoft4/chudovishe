cmake_minimum_required(VERSION 3.8)
project(chudovishe_serial_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ВАЖНО: serial может быть "catkin-style", поэтому подключаем вручную include/lib.
find_package(serial REQUIRED)

add_executable(serial_node
  src/chudovishe_serial_test.cpp
)

target_compile_features(serial_node PUBLIC cxx_std_17)

ament_target_dependencies(serial_node
  rclcpp
  std_msgs
)

# --- Подключение serial ---
# 1) Если пакет экспортирует include dirs переменной
if(DEFINED serial_INCLUDE_DIRS)
  target_include_directories(serial_node PRIVATE ${serial_INCLUDE_DIRS})
endif()

# 2) Если пакет экспортирует библиотеку переменной
if(DEFINED serial_LIBRARIES)
  target_link_libraries(serial_node ${serial_LIBRARIES})
endif()

# 3) Если пакет экспортирует CMake target (иногда бывает)
if(TARGET serial::serial)
  target_link_libraries(serial_node serial::serial)
elseif(TARGET serial)
  target_link_libraries(serial_node serial)
endif()

install(TARGETS serial_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
