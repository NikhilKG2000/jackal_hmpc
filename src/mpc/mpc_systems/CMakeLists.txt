cmake_minimum_required(VERSION 3.8)
project(mpc_jackal)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(mpc_base REQUIRED)
find_package(mpc_core REQUIRED)
find_package(mpc_modules REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(mpc_solver REQUIRED)
find_package(mpc_tools REQUIRED)
find_package(simple_sim REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/jackal_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wno-deprecated-declarations
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav_msgs
  std_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  mpc_base
  mpc_core
  mpc_modules
  mpc_msgs
  mpc_tools
  simple_sim
)

add_executable(mpc_control_node_2_layer
  src/mpc_control_node_2_layer.cpp
)

target_compile_features(
  mpc_control_node_2_layer PUBLIC cxx_std_14)

target_link_libraries(mpc_control_node_2_layer
  ${PROJECT_NAME}
)

ament_target_dependencies(mpc_control_node_2_layer
  rclcpp
  rcl_interfaces
  mpc_base
  mpc_core
  mpc_modules
  mpc_solver
  mpc_tools
  simple_sim
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS mpc_control_node_2_layer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  Eigen3
  Boost
  geometry_msgs
  nav_msgs
  std_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  mpc_base
  mpc_core
  mpc_modules
  mpc_msgs
  mpc_tools
  simple_sim
)

ament_package()