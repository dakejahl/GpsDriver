#pragma once

#include "PX4-GPSDrivers/src/ubx.h"
#include <serial/serial.h>
#include "definitions.h"

#define GPS_RECEIVE_TIMEOUT 1200
#define SERIAL_TIMEOUT 1000

class GpsClass
{
public:
	GpsClass(const std::string& path);
	~GpsClass();

	void run();

	static int callbackEntry(GPSCallbackType type, void *data1, int data2, void *user);

private:
	int callback(GPSCallbackType type, void *data1, int data2);

	sensor_gps_s _gps_report = {};
	satellite_info_s _satellite_report = {};

	serial::Serial _serial = {};
};