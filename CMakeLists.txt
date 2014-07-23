cmake_minimum_required(VERSION 2.8)
PROJECT(indi_qhy9_libqhyccd_ccd C CXX)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")
SET(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -std=c++0x -g -O0" )
SET(CMAKE_CXX_FLAGS_DEBUG "-Werror" )

find_package(INDI REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})
include_directories("/usr/local/include/libqhyccd")

########### QHY ###########
set(indiqhy_SRCS ${CMAKE_SOURCE_DIR}/src/qhy9.cc
)

add_executable(indi_qhy9_libqhyccd_ccd ${indiqhy_SRCS})

target_link_libraries(indi_qhy9_libqhyccd_ccd
        ${INDI_DRIVER_LIBRARIES}
        qhyccd
)

install(TARGETS indi_qhy9_libqhyccd_ccd RUNTIME DESTINATION bin )

install(FILES indi_qhy9_libqhyccd_ccd.xml DESTINATION ${INDI_DATA_DIR} )
