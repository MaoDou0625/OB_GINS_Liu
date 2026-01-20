cmake_minimum_required(VERSION 3.10)
project(ob_gins)
enable_testing()
# 针对 Windows MSVC 编译器开启 /bigobj 支持
if(MSVC)
    add_compile_options(/bigobj)
endif()
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

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES} Sophus::Sophus)

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

# GTest
find_package(GTest REQUIRED)

# Test Executable (Manual GTest replacement)
add_executable(test_bspline_derivatives test/test_bspline_derivatives.cpp)
target_link_libraries(test_bspline_derivatives ${YAML_CPP_LIBRARIES} Sophus::Sophus GTest::gtest GTest::gtest_main)
add_test(NAME test_bspline_derivatives COMMAND test_bspline_derivatives)

# Example: Spline Fitting
add_executable(test_spline_fitting examples/test_spline_fitting.cpp src/factors/pose_manifold.cc)
target_link_libraries(test_spline_fitting ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Test Physics
add_executable(test_wheel_physics test/test_wheel_physics.cpp)
target_link_libraries(test_wheel_physics ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Test Wheel NHC
add_executable(test_wheel_nhc test/test_wheel_nhc.cpp src/factors/pose_manifold.cc)
target_link_libraries(test_wheel_nhc ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Test NHC Observability
add_executable(test_nhc_observability test/test_nhc_observability.cpp src/factors/pose_manifold.cc)
target_link_libraries(test_nhc_observability ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Test Inertial Factor
add_executable(test_inertial_factor test/test_inertial_factor.cpp src/factors/pose_manifold.cc)
target_link_libraries(test_inertial_factor ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Test Inertial Physics
add_executable(test_inertial_physics test/test_inertial_physics.cpp src/factors/pose_manifold.cc)
target_link_libraries(test_inertial_physics ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Test Bias RW
add_executable(test_bias_rw test/test_bias_rw.cpp)
target_link_libraries(test_bias_rw ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES})

# Test Spline Initializer
add_executable(test_spline_initializer test/test_spline_initializer.cpp)
target_link_libraries(test_spline_initializer ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Example: Short Segment Optimization
add_executable(test_short_segment_opt examples/test_short_segment_opt.cpp src/factors/pose_manifold.cc)
target_link_libraries(test_short_segment_opt ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} Sophus::Sophus)

# Test Continuous GNSS Factor
add_executable(test_continuous_gnss test/test_continuous_gnss.cpp)
target_link_libraries(test_continuous_gnss ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} Sophus::Sophus)













# Test Spline Fitting Analysis (GTest)
add_executable(test_spline_fitting_gtest test/test_spline_fitting.cpp)
target_link_libraries(test_spline_fitting_gtest ${YAML_CPP_LIBRARIES} Sophus::Sophus GTest::gtest GTest::gtest_main)

# Continuous Time OB_GINS (Batch Optimization)
add_executable(ob_gins_ct 
    src/ob_gins_ct.cc
    src/core/imu_processor.cc
    src/common/debug.cc
    src/common/interpolation.cc
    src/fileio/fileloader.cc
    src/fileio/filesaver.cc
    src/factors/pose_manifold.cc
)
target_link_libraries(ob_gins_ct 
    ${CERES_LIBRARIES} 
    ${YAML_CPP_LIBRARIES} 
    Sophus::Sophus 
    glog::glog
    absl::strings
    absl::str_format
    absl::time
)
