cmake_minimum_required(VERSION 3.10)
project(dobot_executive)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dobot_kinematics REQUIRED)
find_package(dobot_motion_planning REQUIRED)
find_package(dobot_trajectory REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

# 1. Build Executive Shared Library
add_library(dobot_executive_lib SHARED
  src/executive_engine.cpp
  src/task_state.cpp
  src/task_state_machine.cpp
)

target_include_directories(dobot_executive_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
  ${dobot_motion_planning_INCLUDE_DIRS}
  ${dobot_trajectory_INCLUDE_DIRS}
)

ament_target_dependencies(dobot_executive_lib
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

target_link_libraries(dobot_executive_lib
  dobot_kinematics::dobot_kinematics_lib
  dobot_motion_planning::dobot_motion_planning_lib
  dobot_trajectory::dobot_trajectory_lib
  Python3::Python
)

# 2. Build Executive Orchestrator Node Executable
add_executable(dobot_executive_node
  src/dobot_executive_node.cpp
)

target_include_directories(dobot_executive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
  ${dobot_motion_planning_INCLUDE_DIRS}
  ${dobot_trajectory_INCLUDE_DIRS}
)

target_link_libraries(dobot_executive_node
  dobot_executive_lib
  dobot_kinematics::dobot_kinematics_lib
  dobot_motion_planning::dobot_motion_planning_lib
  dobot_trajectory::dobot_trajectory_lib
  Python3::Python
)

ament_target_dependencies(dobot_executive_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

# 3. Build Experiment Node Executable (Phase 5 Experiments)
add_executable(dobot_experiment_node
  src/dobot_experiment_node.cpp
)

target_include_directories(dobot_experiment_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
  ${dobot_motion_planning_INCLUDE_DIRS}
  ${dobot_trajectory_INCLUDE_DIRS}
)

target_link_libraries(dobot_experiment_node
  dobot_executive_lib
  dobot_kinematics::dobot_kinematics_lib
  dobot_motion_planning::dobot_motion_planning_lib
  dobot_trajectory::dobot_trajectory_lib
  Python3::Python
)

ament_target_dependencies(dobot_experiment_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

# 4. Build Autonomous Pick-and-Place State Machine Executable (Phase 6)
add_executable(dobot_pick_and_place_node
  src/dobot_pick_and_place_node.cpp
)

target_include_directories(dobot_pick_and_place_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
  ${dobot_motion_planning_INCLUDE_DIRS}
  ${dobot_trajectory_INCLUDE_DIRS}
)

target_link_libraries(dobot_pick_and_place_node
  dobot_executive_lib
  dobot_kinematics::dobot_kinematics_lib
  dobot_motion_planning::dobot_motion_planning_lib
  dobot_trajectory::dobot_trajectory_lib
  Python3::Python
)

ament_target_dependencies(dobot_pick_and_place_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

# 5. Installation
install(
  TARGETS dobot_executive_lib dobot_executive_node dobot_experiment_node dobot_pick_and_place_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 4. Export for downstream packages
ament_export_include_directories(include)
ament_export_libraries(dobot_executive_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
