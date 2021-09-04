/**
 * @file definitions.h
 * common platform-specific definitions & abstractions for gps
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <chrono>
#include "satellite_info.h"
#include <stdio.h>
#include <unistd.h>

#define GPS_INFO(...) printf(__VA_ARGS__)
#define GPS_WARN(...) printf(__VA_ARGS__)
#define GPS_ERR(...) printf(__VA_ARGS__)

#define M_PI				3.14159265358979323846
#define M_PI_F				3.14159265358979323846f
#define M_DEG_TO_RAD 		(M_PI / 180.0)
#define M_RAD_TO_DEG 		(180.0 / M_PI)
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f

#define gps_usleep usleep

typedef uint64_t gps_abstime;

/**
 * Get the current time in us. Function signature:
 * uint64_t hrt_absolute_time()
 */
static inline gps_abstime gps_absolute_time() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch()).count();
}

struct sensor_gps_s {
	uint64_t timestamp;
	uint64_t time_utc_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t alt_ellipsoid;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	uint16_t automatic_gain_control;
	uint8_t jamming_state;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	int32_t timestamp_time_relative;
	float heading;
	float heading_offset;
	uint8_t fix_type;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t _padding0[5]; // required for logger
};