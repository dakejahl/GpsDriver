cmake_minimum_required(VERSION 2.8.12)

project(gps_desktop)

add_definitions("-std=c++11 -Wall -Wextra")

include_directories(. serial/include)

set(SERIAL_SRCS
    serial/src/serial.cc
    serial/src/impl/unix.cc
)

add_library(serial ${SERIAL_SRCS})
target_link_libraries(serial rt)

add_executable(gps
    # Application sources
    main.cpp
    GpsClass.cpp
    # GPS Drivers source
    PX4-GPSDrivers/src/gps_helper.cpp
	PX4-GPSDrivers/src/mtk.cpp
	PX4-GPSDrivers/src/rtcm.cpp
	PX4-GPSDrivers/src/sbf.cpp
	PX4-GPSDrivers/src/ubx.cpp
)

target_include_directories(gps PRIVATE
    PX4-GPSDrivers
)

target_link_libraries(gps
    serial)