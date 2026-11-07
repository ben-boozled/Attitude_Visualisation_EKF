cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Command line interface
# Step 1: Call idf.py fullclean first
# Step 2: Choose the app target: 
# (Default to build ekf_imu_app) idf.py build
# (Alternative to build raw_imu_app) idf.py -DAPP_TARGET=raw_imu_app build
# =============================================================================
if(NOT DEFINED APP_TARGET)
    set(APP_TARGET ekf_imu_app)
endif()

message(STATUS ">>>>> Building Target: ${APP_TARGET} <<<<<")

if(APP_TARGET STREQUAL "raw_imu_app")
    set(EXCLUDE_COMPONENTS EKF) # EKF not required to run raw_imu_app
endif()

# ==============================================================
# Register target directories and initialise ESP-IDF project 
# ===============================================================
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/apps/${APP_TARGET}") # do not add the libs/ directory here to avoid autoprocessing by ESP-IDF build system

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${APP_TARGET})  # trigger compiler detection and toolchain initialisation