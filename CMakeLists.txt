cmake_minimum_required(VERSION 2.8.12)

project(gps_desktop)

add_definitions("-std=c++11 -Wall -Wextra")

# add_definitions(-DMAVLINK_USE_CONVENIENCE_FUNCTIONS -DMAVLINK_SEND_UART_BYTES)
# add_definitions(-Wno-missing-field-initializers)

include_directories(.)

# set(serial_SRCS serial/src/serial.cc serial/include/serial/serial.h serial/include/serial/v8stdint.h serial/src/impl/unix.cc)

# add_library(serial ${serial_SRCS})
# target_link_libraries(serial rt)

add_executable(gps
    # Application sources
    main.cpp
    GpsClass.cpp
    SerialConnection.cpp
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

# target_link_libraries(firefly_rtk
#     serial)