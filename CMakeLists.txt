cmake_minimum_required(VERSION 3.10)
project(ob_gins)

# Use C++20 to support designated initializers on MSVC
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MSVC)
    # Avoid passing GCC/Clang flags like -O3/-Wall to MSVC
    # Force UTF-8 source file encoding to eliminate comment garbling on MSVC
    add_compile_options(/permissive- /utf-8)
else ()
    if (CMAKE_BUILD_TYPE MATCHES "Debug")
        add_compile_options(-Wall)
    else ()
        add_compile_options(-Wall -O3)
    endif ()
endif ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE SOURCE
        src/ob_gins.cc
        src/core/imu_chain.cc
        src/common/debug.cc
        src/common/interpolation.cc
        src/bridge/imu_adapter.cc
        src/fileio/fileloader.cc
        src/fileio/filesaver.cc
        src/factors/pose_manifold.cc
        src/preintegration/preintegration_base.cc
        src/preintegration/preintegration_normal.cc
        src/preintegration/preintegration_earth.cc
        src/preintegration/preintegration_odo.cc
        src/preintegration/preintegration_earth_odo.cc
        # wheel preintegration (independent classes, currently not wired)
        src/wheel/preintegration_wheel_base.cc
        src/wheel/preintegration_wheel_normal.cc
        src/wheel/preintegration_wheel_odo.cc
        src/wheel/preintegration_wheel_earth.cc
        src/wheel/preintegration_wheel_earth_odo.cc)

include_directories(${PROJECT_SOURCE_DIR})
add_executable(${PROJECT_NAME} ${SOURCE})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

# MSVC needs _USE_MATH_DEFINES for M_PI from <cmath>
if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif ()

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(src/thirdparty/abseil-cpp abseil-cpp)
target_link_libraries(${PROJECT_NAME}
        absl::strings
        absl::str_format
        absl::time)

# Ceres
find_package(Ceres REQUIRED)
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
