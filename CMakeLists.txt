set(HAKO_CMAKE_VERSION 3.5.1)
cmake_minimum_required(VERSION ${HAKO_CMAKE_VERSION})

project(hakoniwa-px4sim
    VERSION 1.0.0
    DESCRIPTION "hakoniwa-px4sim project"
    LANGUAGES C CXX
)

enable_testing()

set(MAVLINK_SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/c_library_v2")
set(HAKONIWA_CORE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/hakoniwa-core-cpp-client/core/src")
set(HAKONIWA_SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/hakoniwa-core-cpp-client/src/hakoc/src")
set(HAKONIWA_PDU_SOURCE_DIR "${PROJECT_SOURCE_DIR}/third-party/hakoniwa-ros2pdu/pdu/types")

if (DEFINED HAKO_CLIENT_OPTION_FILEPATH)
    include(${HAKO_CLIENT_OPTION_FILEPATH})
else()
    include(default-cmake-options.cmake)
endif()

message(STATUS "BUILD_TYPE" ${BUILD_TYPE})
message(STATUS "GCOV" ${GCOV})

add_subdirectory(third-party/hakoniwa-core-cpp-client)
add_subdirectory(src)
