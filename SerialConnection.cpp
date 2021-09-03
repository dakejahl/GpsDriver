#include "SerialConnection.hpp"
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#include <string>


#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include<signal.h>

#define GET_ERROR() strerror(errno)


SerialConnection::SerialConnection(const std::string& path, int baudrate)
    : _serial_port_path(path)
    , _baudrate(baudrate)
{
}

SerialConnection::~SerialConnection()
{
    // If no one explicitly called stop before, we should at least do it.
    stop();
}

ConnectionResult SerialConnection::start()
{
    ConnectionResult ret = setup_port();
    if (ret != ConnectionResult::Success) {
        return ret;
    }

    return ConnectionResult::Success;
}

size_t SerialConnection::read_bytes(void* data, size_t size)
{
    int recv_len = static_cast<int>(read(_fd, data, size));

    return recv_len;
}

size_t SerialConnection::write_bytes(void* data, size_t size)
{
    return write(_fd, data, size);
}

ConnectionResult SerialConnection::setup_port()
{
    std::cout << "Setting up serial port" << std::endl;
    // open() hangs on macOS or Linux devices(e.g. pocket beagle) unless you give it O_NONBLOCK
    _fd = open(_serial_port_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd == -1) {
        std::cout << "open failed: " << GET_ERROR() << std::endl;
        return ConnectionResult::ConnectionError;
    }
    // We need to clear the O_NONBLOCK again because we can block while reading
    // as we do it in a separate thread.
    if (fcntl(_fd, F_SETFL, 0) == -1) {
        std::cout << "fcntl failed: " << GET_ERROR() << std::endl;
        return ConnectionResult::ConnectionError;
    }

    struct termios tc;
    bzero(&tc, sizeof(tc));

    if (tcgetattr(_fd, &tc) != 0) {
        std::cout << "tcgetattr failed: " << GET_ERROR() << std::endl;
        close(_fd);
        return ConnectionResult::ConnectionError;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
    tc.c_cflag |= CS8;

    tc.c_cc[VMIN] = 0; // We are ok with 0 bytes.
    tc.c_cc[VTIME] = 10; // Timeout after 1 second.

    tc.c_cflag |= CLOCAL; // Without this a write() blocks indefinitely.

    const int baudrate = define_from_baudrate(_baudrate);

    if (baudrate == -1) {
        return ConnectionResult::BaudrateUnknown;
    }

    if (cfsetispeed(&tc, baudrate) != 0) {
        std::cout << "cfsetispeed failed: " << GET_ERROR() << std::endl;
        close(_fd);
        return ConnectionResult::ConnectionError;
    }

    if (cfsetospeed(&tc, baudrate) != 0) {
        std::cout << "cfsetospeed failed: " << GET_ERROR() << std::endl;
        close(_fd);
        return ConnectionResult::ConnectionError;
    }

    if (tcsetattr(_fd, TCSANOW, &tc) != 0) {
        std::cout << "tcsetattr failed: " << GET_ERROR() << std::endl;
        close(_fd);
        return ConnectionResult::ConnectionError;
    }


    return ConnectionResult::Success;
}

void SerialConnection::set_baudrate(int baud)
{
    struct termios tc;
    bzero(&tc, sizeof(tc));

    if (tcgetattr(_fd, &tc) != 0) {
        std::cout << "tcgetattr failed: " << GET_ERROR() << std::endl;
        close(_fd);
        return;
    }

    const int baudrate = define_from_baudrate(baud);

    if (baudrate == -1) {
        return;
    }

    if (cfsetispeed(&tc, baudrate) != 0) {
        std::cout << "cfsetispeed failed: " << GET_ERROR() << std::endl;
        return;
    }

    if (cfsetospeed(&tc, baudrate) != 0) {
        std::cout << "cfsetospeed failed: " << GET_ERROR() << std::endl;
        return;
    }
}

ConnectionResult SerialConnection::stop()
{
    close(_fd);

    return ConnectionResult::Success;
}

int SerialConnection::define_from_baudrate(int baudrate)
{
    switch (baudrate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default: {
            std::cout << "Unknown baudrate" << std::endl;
            return -1;
        }
    }
}
