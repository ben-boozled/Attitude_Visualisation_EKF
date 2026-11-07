idf_component_register(SRCS "EKF.cpp"
                       INCLUDE_DIRS "include") 

# Handle third-party library downloads here because only the EKF component 
# requires Eigen for matrix operations
set(EIGEN_VERSION 5.0.0)
set(EIGEN_INSTALL_DIR "${CMAKE_SOURCE_DIR}/libs/Eigen") # ../../libs/Eigen

include(ExternalProject)

if (NOT EXISTS ${EIGEN_INSTALL_DIR})
    message(STATUS ">>>>> Downloading Eigen v${EIGEN_VERSION}... <<<<<")
    set(EIGEN_URL URL "https://gitlab.com/libeigen/eigen/-/archive/5.0.0/eigen-${EIGEN_VERSION}.tar.gz")
else()
    message(STATUS ">>>>> Eigen v${EIGEN_VERSION} found installed in ${EIGEN_INSTALL_DIR} <<<<<") 
    set(EIGEN_URL "")   # leave URL empty to avoid downloads if Eigen already exists in libs/
endif()

ExternalProject_Add(eigen 
    ${EIGEN_URL} # dynamic URL field based on whether Eigen has already been installed
    SOURCE_DIR "${EIGEN_INSTALL_DIR}"

    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND "")

# Need to be PUBLIC because EKF.hpp has data types from Eigen 
# (i.e. any header including EKF.hpp will need those data type definitions)
target_include_directories(${COMPONENT_LIB} PUBLIC ${EIGEN_INSTALL_DIR}) 
add_dependencies(${COMPONENT_LIB} eigen)