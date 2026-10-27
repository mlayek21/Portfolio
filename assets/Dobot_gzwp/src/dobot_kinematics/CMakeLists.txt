cmake_minimum_required(VERSION 3.10)
project(dobot_kinematics)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# 1. Build Pure C++ Kinematics Shared Library
add_library(dobot_kinematics_lib SHARED
  src/kinematics_engine.cpp
)

target_include_directories(dobot_kinematics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 2. Build Test Validation Executable
add_executable(test_kinematics_node
  src/test_kinematics_node.cpp
)

target_link_libraries(test_kinematics_node
  dobot_kinematics_lib
)

# 3. Installation
install(
  TARGETS dobot_kinematics_lib test_kinematics_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 4. Export for downstream packages (dobot_trajectory, dobot_motion_planning)
ament_export_include_directories(include)
ament_export_libraries(dobot_kinematics_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()

