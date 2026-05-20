cmake_minimum_required(VERSION 3.8)
project(network_aware_broker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(network_aware_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/time_broker.cpp
  src/metrics_estimator.cpp
  src/telemetry_fuser.cpp
  src/pose_cache.cpp
  src/broker_node.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  std_msgs
  network_aware_msgs
)

add_executable(broker_main src/broker_main.cpp)
target_link_libraries(broker_main ${PROJECT_NAME})
ament_target_dependencies(broker_main rclcpp)

add_executable(ue_ingress_node src/ue_ingress_node.cpp)
target_link_libraries(ue_ingress_node ${PROJECT_NAME})
ament_target_dependencies(ue_ingress_node rclcpp network_aware_msgs)

add_executable(metrics_ingress_node src/metrics_ingress_node.cpp)
target_link_libraries(metrics_ingress_node ${PROJECT_NAME})
ament_target_dependencies(metrics_ingress_node rclcpp network_aware_msgs)

add_executable(rtf_clock_node src/rtf_clock_node.cpp)
target_link_libraries(rtf_clock_node ${PROJECT_NAME})
ament_target_dependencies(rtf_clock_node rclcpp rosgraph_msgs network_aware_msgs)

install(
  TARGETS
    ${PROJECT_NAME}
    broker_main
    ue_ingress_node
    metrics_ingress_node
    rtf_clock_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  std_msgs
  network_aware_msgs
)

ament_package()
