cmake_minimum_required(VERSION 3.16)
project(rolling_bonxai_ros)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# PCL and Eigen have no ament packages, so ament_auto_find_build_dependencies()
# skips their rosdep keys (`libpcl-all-dev`, `eigen`). Find them explicitly.
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

# ============================================================================
# RollingBonxaiServer library (component + standalone executable)
# ============================================================================

ament_auto_add_library(rolling_bonxai_server_lib SHARED
  src/rolling_bonxai_server.cpp
)

target_compile_features(rolling_bonxai_server_lib PUBLIC cxx_std_17)

# PCL leaks -march=native style flags and definitions through PCL_DEFINITIONS;
# include dirs are marked SYSTEM so PCL's own warnings do not fail our build.
target_include_directories(rolling_bonxai_server_lib SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})

# Plain signature: ament_auto_add_library() already links with the plain form,
# and CMake forbids mixing the keyword and plain signatures on one target.
target_link_libraries(rolling_bonxai_server_lib
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Register as composable node (for component containers)
rclcpp_components_register_node(rolling_bonxai_server_lib
  PLUGIN "RollingBonxai::RollingBonxaiServer"
  EXECUTABLE rolling_bonxai_server_component
)

# Standalone executable
ament_auto_add_executable(rolling_bonxai_server_node
  src/main.cpp
)
target_link_libraries(rolling_bonxai_server_node rolling_bonxai_server_lib)

ament_export_dependencies(PCL Eigen3)

# ============================================================================
# Testing
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# USE_SCOPED_HEADER_INSTALL_DIR installs headers into include/${PROJECT_NAME},
# which is the ament_auto default from Kilted onwards. Passing it explicitly
# keeps the installed layout identical on Jazzy and Lyrical.
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    launch
    params
)
