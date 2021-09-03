#include <GpsClass.hpp>


int GpsClass::callbackEntry(GPSCallbackType type, void* data1, int data2, void* user)
{
    auto gps = reinterpret_cast<GpsClass*>(user);

    return gps->callback(type, data1, data2);
}

ConnectionResult GpsClass::setup_serial_connection(const std::string& path, int baudrate)
{
    _serial = new SerialConnection(path, baudrate);

    return _serial->start();
}

void GpsClass::run()
{
	GPSDriverUBX* gpsDriver = new GPSDriverUBX(GPSDriverUBX::Interface::UART, &callbackEntry, this, &_gps_report, &_satellite_report);

	// gpsDriver->setSurveyInSpecs(SURVEYINACCMETERS * 10000.0f, MINIMUMOBSERVATIONTIME);

	unsigned int auto_baudrate = 0;

	if (!gpsDriver->configure(auto_baudrate, GPSDriverUBX::OutputMode::GPS) == 0) {
		std::cout << "GPS Configure Error" << std::endl;
		return;
	}

	// In rare cases it can happen that we get an error from the driver (eg. checksum failure) due to
	// bus errors or buggy firmware. In this case we want to try multiple times before giving up.
	int numTries = 0;

	while (numTries < 3) {

		int ret = gpsDriver->receive(GPS_RECEIVE_TIMEOUT);

		if (ret > 0) {
			numTries = 0;

			 // bit 0 set: got gps position update
			if (ret & 1) {

				std::cout << "GPS Position" << std::endl;
				std::cout << "timestamp: " << _gps_report.timestamp << std::endl;
				std::cout << "lat: " << _gps_report.lat << std::endl;
				std::cout << "lon: " << _gps_report.lon << std::endl;
			}

			 // bit 1 set: got satellite info update
			if (ret & 2) {
				std::cout << "Publish Satellite Info" << std::endl;
			}

		} else {
			std::cout << "error! ret : " << ret << std::endl;
			++numTries;
		}
	}

	std::cout << "GpsClass run() exiting" << std::endl;

    return;
}

int GpsClass::callback(GPSCallbackType type, void* data1, int data2)
{
	switch (type) {
	case GPSCallbackType::readDeviceData:
		// TODO: do we need to check if there is data available?
		return _serial->read_bytes(data1, data2);

	case GPSCallbackType::writeDeviceData:
		return _serial->write_bytes(data1, data2);

	case GPSCallbackType::setBaudrate:
		std::cout << "GPSCallbackType::setBaudrate :" << data2 << std::endl;
		_serial->set_baudrate(data2);
		return 0;

	default:
		std::cout << "Default case statement -- do nothing";
	}


	// case GPSCallbackType::gotRTCMMessage:
	// {
	// 	std::cout << "GPSCallbackType::gotRTCMMessage" << std::endl;
	// 	// gotRTCMData((uint8_t*) data1, data2);
	// 	break;
	// }

	// case GPSCallbackType::surveyInStatus:
	// {
	// 	SurveyInStatus* status = (SurveyInStatus*)data1;
	// 	std::cout << "surveyInStatus: Lat: " << status->latitude << " Lon: " << status->longitude << " Alt: " << status->altitude << "m Accur: " << status->mean_accuracy << "mm Duration: " << status->duration << std::endl;
	// 	break;
	// }

	// case GPSCallbackType::setClock:
	// 	// not used
	// 	//std::cout << "GPSCallbackType::setClock" << std::endl;
	// 	break;
	// }

	return 0;
}