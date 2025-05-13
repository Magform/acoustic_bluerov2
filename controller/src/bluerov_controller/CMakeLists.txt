cmake_minimum_required(VERSION 3.8)
project(bluerov_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SDL2 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
)

add_executable(xbox
  
  src/main.cpp
  src/controller_reader.cpp
  src/publisher_node.cpp
)

ament_target_dependencies(xbox
  rclcpp
  std_msgs
)

target_link_libraries(xbox
  ${SDL2_LIBRARIES}
  yaml-cpp
)

install(TARGETS xbox
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()