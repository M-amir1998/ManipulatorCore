cmake_minimum_required(VERSION 3.22)
project(manipulatorcore_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Installed ManipulatorCore package (pass its prefix through CMAKE_PREFIX_PATH).
find_package(ManipulatorCore 0.1 CONFIG REQUIRED)

# ---------------------------------------------------------------------------------------------
# Conversion library shared by the executables and the tests
# ---------------------------------------------------------------------------------------------
add_library(manipulatorcore_ros_conversions SHARED src/conversions.cpp src/robots.cpp)
target_include_directories(
  manipulatorcore_ros_conversions PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_features(manipulatorcore_ros_conversions PUBLIC cxx_std_20)
target_link_libraries(
  manipulatorcore_ros_conversions
  PUBLIC ManipulatorCore::planning
         ManipulatorCore::model
         ManipulatorCore::trajectory
         ManipulatorCore::geometry
         ${builtin_interfaces_TARGETS}
         ${geometry_msgs_TARGETS}
         ${sensor_msgs_TARGETS}
         ${std_msgs_TARGETS}
         ${trajectory_msgs_TARGETS}
         ${visualization_msgs_TARGETS})

# ---------------------------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------------------------
add_executable(export_urdf src/export_urdf.cpp)
target_link_libraries(export_urdf PRIVATE manipulatorcore_ros_conversions)

add_executable(manipulatorcore_simulator src/simulator_node.cpp)
target_link_libraries(manipulatorcore_simulator PRIVATE manipulatorcore_ros_conversions ManipulatorCore::simulation
                                                        ManipulatorCore::control rclcpp::rclcpp)

add_executable(manipulatorcore_planner src/planner_node.cpp)
target_link_libraries(manipulatorcore_planner PRIVATE manipulatorcore_ros_conversions ManipulatorCore::planning
                                                      ManipulatorCore::kinematics rclcpp::rclcpp)

# ---------------------------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS manipulatorcore_ros_conversions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS export_urdf manipulatorcore_simulator manipulatorcore_planner DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions manipulatorcore_ros_conversions)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  ManipulatorCore)

ament_package()
