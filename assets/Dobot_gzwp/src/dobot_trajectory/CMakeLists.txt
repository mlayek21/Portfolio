cmake_minimum_required(VERSION 3.10)
project(dobot_trajectory)

# Enforce C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# macOS @rpath support
set(CMAKE_MACOSX_RPATH ON)
set(CMAKE_INSTALL_RPATH "@loader_path/..;${CMAKE_INSTALL_PREFIX}/lib;$ENV{CONDA_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dobot_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

# 1. Build Trajectory Planning Shared Library
add_library(dobot_trajectory_lib SHARED
  src/trajectory_planning_engine.cpp
)

target_include_directories(dobot_trajectory_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
)

ament_target_dependencies(dobot_trajectory_lib
  rclcpp
  trajectory_msgs
)

target_link_libraries(dobot_trajectory_lib
  dobot_kinematics::dobot_kinematics_lib
  Python3::Python
)

# 2. Build Trajectory Verification Node
add_executable(test_trajectory_node
  src/test_trajectory_node.cpp
)

target_include_directories(test_trajectory_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
)

target_link_libraries(test_trajectory_node
  dobot_trajectory_lib
  dobot_kinematics::dobot_kinematics_lib
  Python3::Python
)

ament_target_dependencies(test_trajectory_node
  rclcpp
  trajectory_msgs
)

# 3. Installation
install(
  TARGETS dobot_trajectory_lib test_trajectory_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 4. Export for downstream packages
ament_export_include_directories(include)
ament_export_libraries(dobot_trajectory_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
