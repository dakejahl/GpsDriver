#pragma once

#include <mutex>
#include <iostream>

enum class ConnectionResult {
    Success = 0, /**< @brief %Connection succeeded. */
    Timeout, /**< @brief %Connection timed out. */
    SocketError, /**< @brief Socket error. */
    BindError, /**< @brief Bind error. */
    SocketConnectionError, /**< @brief Socket connection error. */
    ConnectionError, /**< @brief %Connection error. */
    NotImplemented, /**< @brief %Connection type not implemented. */
    SystemNotConnected, /**< @brief No system is connected. */
    SystemBusy, /**< @brief %System is busy. */
    CommandDenied, /**< @brief Command is denied. */
    DestinationIpUnknown, /**< @brief %Connection IP is unknown. */
    ConnectionsExhausted, /**< @brief %Connections exhausted. */
    ConnectionUrlInvalid, /**< @brief URL invalid. */
    BaudrateUnknown /**< @brief Baudrate unknown. */
};

class SerialConnection {
public:
    SerialConnection(const std::string& path, int baudrate);
    ~SerialConnection();

    ConnectionResult start();
    ConnectionResult stop();

    size_t read_bytes(void* data, size_t size);
    size_t write_bytes(void* data, size_t size);

    void set_baudrate(int baud);

    // Non-copyable
    // SerialConnection(const SerialConnection&) = delete;
    // const SerialConnection& operator=(const SerialConnection&) = delete;

private:
    ConnectionResult setup_port();

    static int define_from_baudrate(int baudrate);

    const std::string _serial_port_path;
    const int _baudrate;

    std::mutex _mutex = {};

    int _fd = -1;
};

