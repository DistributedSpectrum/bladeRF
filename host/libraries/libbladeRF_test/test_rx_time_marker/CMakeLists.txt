cmake_minimum_required(VERSION 3.10...3.27)
project(libbladeRF_test_rx_time_marker C)

set(INCLUDES
    ${libbladeRF_SOURCE_DIR}/include
    ${BLADERF_HOST_COMMON_INCLUDE_DIRS}
)

if(MSVC)
    set(INCLUDES ${INCLUDES} ${MSVC_C99_INCLUDES})
endif()

set(LIBS libbladerf_shared)

set(SRC
        main.c
)
if(MSVC)
    set(SRC ${SRC}
        ${BLADERF_HOST_COMMON_SOURCE_DIR}/windows/getopt_long.c
    )
endif()

include_directories(${INCLUDES})
add_executable(libbladeRF_test_rx_time_marker ${SRC})
target_link_libraries(libbladeRF_test_rx_time_marker ${LIBS} m)
