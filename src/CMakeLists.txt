set(HAKO_CMAKE_VERSION ${HAKO_CMAKE_VERSION})

project(hakoniwa-px4sim
    LANGUAGES C CXX
)

set(OS_TYPE "posix")
if(WIN32)
    set(OS_TYPE "win")
endif(WIN32)
MESSAGE(STATUS "OS_TYPE=" ${OS_TYPE})

add_executable(
    hako-px4sim
    comm/udp_connector.cpp
    mavlink/mavlink_decoder.cpp
    mavlink/mavlink_encoder.cpp
    px4sim_main.cpp
)

target_include_directories(
    hako-px4sim
    PRIVATE /usr/local/include
    PRIVATE /mingw64/include
    PRIVATE ${MAVLINK_SOURCE_DIR}/all
    PRIVATE ${PROJECT_SOURCE_DIR}/src
)
