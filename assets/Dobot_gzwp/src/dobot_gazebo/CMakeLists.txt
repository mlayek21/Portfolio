cmake_minimum_required(VERSION 3.8)
project(dobot_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dobot_description REQUIRED)
find_package(dobot_kinematics REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

# macOS @rpath support
set(CMAKE_MACOSX_RPATH ON)
set(CMAKE_INSTALL_RPATH "@loader_path/..;${CMAKE_INSTALL_PREFIX}/lib;$ENV{CONDA_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Build C++ Simulation Node
add_executable(dobot_sim_node src/dobot_sim_node.cpp)

target_include_directories(dobot_sim_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
)

ament_target_dependencies(dobot_sim_node
  rclcpp
  dobot_description
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  trajectory_msgs
)
target_link_libraries(dobot_sim_node
  dobot_kinematics::dobot_kinematics_lib
  Python3::Python
)

# Install executable and directories
install(TARGETS
  dobot_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
