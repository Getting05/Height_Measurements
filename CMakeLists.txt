cmake_minimum_required(VERSION 3.8)
project(height_measurements)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(height_measurement_node
  src/height_measurement_node.cpp
  src/npy_reader.cpp
)

target_link_libraries(height_measurement_node
  yaml-cpp
)

ament_target_dependencies(height_measurement_node
  rclcpp
  std_msgs
  nav_msgs
)

install(TARGETS height_measurement_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch tools
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
