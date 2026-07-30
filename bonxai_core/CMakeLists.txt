cmake_minimum_required(VERSION 3.16)
project(bonxai_core)

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

# Header-only library - use INTERFACE
ament_auto_add_library(${PROJECT_NAME} INTERFACE)

# Require C++17
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

# Testing
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
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
