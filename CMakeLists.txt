cmake_minimum_required(VERSION 3.8)
project(webots_mini_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(DEFINED ENV{WEBOTS_HOME})
  set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
  set(WEBOTS_HOME /snap/webots/current/usr/share/webots)
endif()

set(WEBOTS_INCLUDE
  ${WEBOTS_HOME}/include/controller/cpp
  ${WEBOTS_HOME}/include/controller/cpp/utils
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(ROS_VERSION VERSION_GREATER_EQUAL "humble")
  message(STATUS "ROS 2 Humble or later detected")
else()
  message(STATUS "ROS 2 version is older than Humble")

  find_library(WEBOTS_CONTROLLER_LIB CppController ${WEBOTS_HOME}/lib/controller)

  add_executable(webots_node src/webots_node.cc)

  target_include_directories(webots_node PRIVATE ${WEBOTS_INCLUDE})
  target_link_libraries(webots_node ${WEBOTS_CONTROLLER_LIB})

  ament_target_dependencies(webots_node
    rclcpp
    geometry_msgs
    nav_msgs
  )

  install(TARGETS
    webots_node
    DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY
  launch worlds resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
