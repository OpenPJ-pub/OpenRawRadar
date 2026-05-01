#pragma once

#include <stdexcept>
#include <string>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <mutex>

namespace radar_socket_platform {

using socket_handle_t = SOCKET;
using sockaddr_length_t = int;

inline constexpr socket_handle_t kInvalidSocket = INVALID_SOCKET;

inline void ensure_socket_runtime() {
    static std::once_flag winsock_once;
    std::call_once(winsock_once, []() {
        WSADATA winsock_data{};
        if (WSAStartup(MAKEWORD(2, 2), &winsock_data) != 0) {
            throw std::runtime_error("Failed to initialize WinSock");
        }
    });
}

inline int last_socket_error_code() {
    return WSAGetLastError();
}

inline std::string socket_error_message(const int error_code) {
    return "WinSock error " + std::to_string(error_code);
}

inline std::string last_socket_error_message() {
    return socket_error_message(last_socket_error_code());
}

inline socket_handle_t create_udp_socket_handle() {
    ensure_socket_runtime();
    return socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
}

inline void close_socket_handle(socket_handle_t& socket_handle) {
    if (socket_handle != kInvalidSocket) {
        closesocket(socket_handle);
        socket_handle = kInvalidSocket;
    }
}

inline bool set_socket_nonblocking(const socket_handle_t socket_handle) {
    u_long mode = 1;
    return ioctlsocket(socket_handle, FIONBIO, &mode) == 0;
}

inline int recvfrom_socket(const socket_handle_t socket_handle,
                           char* buffer,
                           const int buffer_size,
                           sockaddr* source_address,
                           sockaddr_length_t* source_address_length) {
    return recvfrom(socket_handle, buffer, buffer_size, 0, source_address, source_address_length);
}

inline bool socket_would_block() {
    return WSAGetLastError() == WSAEWOULDBLOCK;
}

}  // namespace radar_socket_platform

#else

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace radar_socket_platform {

using socket_handle_t = int;
using sockaddr_length_t = socklen_t;

inline constexpr socket_handle_t kInvalidSocket = -1;

inline int last_socket_error_code() {
    return errno;
}

inline std::string socket_error_message(const int error_code) {
    return std::strerror(error_code);
}

inline std::string last_socket_error_message() {
    return socket_error_message(last_socket_error_code());
}

inline socket_handle_t create_udp_socket_handle() {
    return socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
}

inline void close_socket_handle(socket_handle_t& socket_handle) {
    if (socket_handle >= 0) {
        close(socket_handle);
        socket_handle = kInvalidSocket;
    }
}

inline bool set_socket_nonblocking(const socket_handle_t socket_handle) {
    const int current_flags = fcntl(socket_handle, F_GETFL, 0);
    if (current_flags == -1) {
        return false;
    }
    return fcntl(socket_handle, F_SETFL, current_flags | O_NONBLOCK) != -1;
}

inline int recvfrom_socket(const socket_handle_t socket_handle,
                           char* buffer,
                           const int buffer_size,
                           sockaddr* source_address,
                           sockaddr_length_t* source_address_length) {
    return recvfrom(socket_handle, buffer, buffer_size, 0, source_address, source_address_length);
}

inline bool socket_would_block() {
    return errno == EWOULDBLOCK || errno == EAGAIN;
}

}  // namespace radar_socket_platform

#endif
