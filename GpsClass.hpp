#pragma once

#include <SerialConnection.hpp>
#include "PX4-GPSDrivers/src/ubx.h"

#include "definitions.h"

#define GPS_RECEIVE_TIMEOUT 1200

class GpsClass
{
public:
	void run();
	ConnectionResult setup_serial_connection(const std::string& path, int baudrate);

	static int callbackEntry(GPSCallbackType type, void *data1, int data2, void *user);

private:
	int callback(GPSCallbackType type, void *data1, int data2);

	sensor_gps_s _gps_report = {};
	satellite_info_s _satellite_report = {};

	SerialConnection* _serial = nullptr;
};