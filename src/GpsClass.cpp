#include <GpsClass.hpp>

GpsClass::GpsClass(const std::string& path, const uint32_t baudrate)
	: _baudrate(baudrate)
	, _path(path)
{}

GpsClass::~GpsClass()
{
	_serial.close();
}

int GpsClass::callbackEntry(GPSCallbackType type, void* data1, int data2, void* user)
{
	auto gps = reinterpret_cast<GpsClass*>(user);
	return gps->callback(type, data1, data2);
}

void GpsClass::initialize()
{
	_serial.setPort(_path);
	_serial.setBaudrate(_baudrate);
	_serial.open();
	_serial.flush();
}

void GpsClass::run()
{
	GPSDriverUBX* gpsDriver = new GPSDriverUBX(GPSDriverUBX::Interface::UART, &callbackEntry, this, &_gps_report, &_satellite_report);

	unsigned autobaud = 0; // Baud of zero means it will try all

	GPSHelper::GPSConfig config = {GPSHelper::OutputMode::GPS, GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS,
				       GPSHelper::InterfaceProtocolsMask::ALL_DISABLED
				      };

	if (!(gpsDriver->configure(autobaud, config) == 0)) {
		printf("GPS Configure Error\n");
		return;
	}

	int retries = 0;

	while (retries < 3) {

		int ret = gpsDriver->receive(GPS_RECEIVE_TIMEOUT);

		if (ret > 0) {
			retries = 0;

			// bit 0: gps position update
			if (ret & 1) {

				printf("Position Update\n");
				printf("timestamp: %zu\n", _gps_report.timestamp);
				printf("lat: %d\n", _gps_report.lat);
				printf("lon: %d\n", _gps_report.lon);
			}

			// bit 1: satellite info update
			if (ret & 2) {
				printf("Satellite Info\n");
				printf("timestamp: %zu\n", _satellite_report.timestamp);
				printf("sat count: %d\n", _satellite_report.count);
			}

		} else {
			printf("error! ret: %d\n", ret);
			++retries;
		}
	}

	printf("GpsClass run() exiting\n");

	return;
}

int GpsClass::callback(GPSCallbackType type, void* data1, int data2)
{
	switch (type) {
	case GPSCallbackType::readDeviceData: {
		if (_serial.waitReadable()) {
			return _serial.read((uint8_t*) data1, data2);
		}

		return 0;
	}

	case GPSCallbackType::writeDeviceData: {
		size_t bytes_written = _serial.write((uint8_t*) data1, data2);
		_serial.waitByteTimes(data2);
		return bytes_written;
	}

	case GPSCallbackType::setBaudrate: {
		printf("GPSCallbackType::setBaudrate: %d\n", data2);
		_serial.setBaudrate(data2);
		return 0;
	}

	default:
		printf("Default case statement\n");
	}

	return 0;
}