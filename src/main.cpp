#include <GpsClass.hpp>

int main(int argv, const char** argc)
{
	(void)argv;
	(void)argc;

	auto gps = new GpsClass("/dev/ttyUSB0", 9600);

	gps->initialize();

	gps->run();

	return 0;
}

