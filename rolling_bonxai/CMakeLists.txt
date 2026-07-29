cmake_minimum_required(VERSION 3.16)
project(rolling_bonxai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Find dependencies
# ============================================================================

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Neither Eigen nor yaml-cpp ships an ament package, so
# ament_auto_find_build_dependencies() skips their rosdep keys. Find them here.
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# AsyncChunkManager spawns std::thread workers.
find_package(Threads REQUIRED)

# yaml-cpp 0.8 (Ubuntu 24.04) exports yaml-cpp::yaml-cpp; older releases only
# defined the unnamespaced `yaml-cpp` target. Normalise to one name.
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# ============================================================================
# Create the library
# ============================================================================

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/common.cpp
  src/coordinate_system.cpp
  src/chunk_storage.cpp
  src/transition_manager.cpp
  src/chunk_loading_policies.cpp
  src/file_storage_backend.cpp
  src/async_chunk_manager.cpp
  src/rolling_occupancy_map.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Plain signature: ament_auto_add_library() already calls
# ament_target_dependencies(), which links with the plain signature, and CMake
# forbids mixing the two forms on one target.
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${YAML_CPP_TARGET}
  Threads::Threads
)

ament_export_dependencies(Eigen3 yaml-cpp)

# ============================================================================
# Testing
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Each test links the shared library directly; the previous duplicate static
  # "testlib" compiled the same sources a second time and drifted out of sync
  # with the real target's source list.
  set(${PROJECT_NAME}_TESTS
    test_common
    test_coordinate_system
    test_file_storage_backend
    test_chunk_storage
    test_chunk_loading_policy
    test_transition_manager
  )

  foreach(_test ${${PROJECT_NAME}_TESTS})
    ament_add_gtest(${_test} test/${_test}.cpp)
    target_link_libraries(${_test} ${PROJECT_NAME})
  endforeach()
endif()

# ============================================================================
# Optional: Build Benchmarks
# ============================================================================

option(BUILD_BENCHMARKS "Build benchmark executables" OFF)

if(BUILD_BENCHMARKS)
  message(STATUS "Building benchmarks (BUILD_BENCHMARKS=ON)")
  add_subdirectory(benchmark)
else()
  message(STATUS "Skipping benchmarks (BUILD_BENCHMARKS=OFF)")
endif()

# ============================================================================
# Package
# ============================================================================

# USE_SCOPED_HEADER_INSTALL_DIR installs headers into include/${PROJECT_NAME},
# which is the ament_auto default from Kilted onwards. Passing it explicitly
# keeps the installed layout identical on Jazzy and Lyrical.
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
