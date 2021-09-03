#include <GpsClass.hpp>

int main(int argv, const char** argc)
{
	(void)argv;
	(void)argc;

	auto gps = new GpsClass();

	std::string path = "/dev/ttyUSB0";

	ConnectionResult ret = gps->setup_serial_connection(path, 115200);

	if (ret != ConnectionResult::Success) {
		std::cout << "Failed to open serial port: " << path;
		return -1;
	}


	gps->run();

	return 0;
}

