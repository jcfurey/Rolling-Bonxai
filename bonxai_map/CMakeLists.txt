cmake_minimum_required(VERSION 3.16)
project(bonxai_map)

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

# Eigen ships no ament package, so ament_auto_find_build_dependencies() silently
# skips the `eigen` rosdep key. Find it explicitly and link the imported target.
find_package(Eigen3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/occupancy_map.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Plain signature: ament_auto_add_library() already calls
# ament_target_dependencies(), which links with the plain signature, and CMake
# forbids mixing the two forms on one target.
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

ament_export_dependencies(Eigen3)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_occupancy_map
    test/test_occupancy_map.cpp
  )
  target_link_libraries(test_occupancy_map ${PROJECT_NAME})
endif()

# USE_SCOPED_HEADER_INSTALL_DIR installs headers into include/${PROJECT_NAME},
# which is the ament_auto default from Kilted onwards. Passing it explicitly
# keeps the installed layout identical on Jazzy and Lyrical.
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
