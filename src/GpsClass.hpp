#pragma once

#include <ubx.h>
#include <serial/serial.h>
#include <definitions.h>

#define GPS_RECEIVE_TIMEOUT 1000

class GpsClass
{
public:
	GpsClass(const std::string& path, const uint32_t baudrate);
	~GpsClass();

	void initialize();
	void run();

	static int callbackEntry(GPSCallbackType type, void* data1, int data2, void* user);

private:
	int callback(GPSCallbackType type, void* data1, int data2);

	sensor_gps_s _gps_report = {};
	satellite_info_s _satellite_report = {};

	serial::Serial _serial = {};

	uint32_t _baudrate {};
	std::string _path {};
};