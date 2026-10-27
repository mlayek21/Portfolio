cmake_minimum_required(VERSION 3.10)
project(dobot_motion_planning)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dobot_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)

# 1. Build Motion Planning Shared Library
add_library(dobot_motion_planning_lib SHARED
  src/motion_planning_engine.cpp
)

target_include_directories(dobot_motion_planning_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${dobot_kinematics_INCLUDE_DIRS}
)

target_link_libraries(dobot_motion_planning_lib PUBLIC
  dobot_kinematics::dobot_kinematics_lib
)

# 2. Build Planning Verification Node
add_executable(test_planning_node
  src/test_planning_node.cpp
)

target_link_libraries(test_planning_node
  dobot_motion_planning_lib
  dobot_kinematics::dobot_kinematics_lib
)

# 3. Installation
install(
  TARGETS dobot_motion_planning_lib test_planning_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 4. Export for downstream packages (dobot_trajectory)
ament_export_include_directories(include)
ament_export_libraries(dobot_motion_planning_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
