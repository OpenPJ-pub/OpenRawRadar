#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <map>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include "radar_driver_cpp/transport/kfifo/unlock_queue.h"
#include "radar_driver_cpp/transport/socket_platform.h"

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define packetSize_d 1466

#define CONFIG_HEADER "5AA5"
#define CONFIG_FOOTER "AAEE"
#define HEADER_NUM 0xA55A
#define FOOTER_NUM 0xEEAA
#define MAX_PACKET_SIZE 4096

#define MAX_BYTES_PER_PACKET  1470
#define FPGA_CLK_CONVERSION_FACTOR  1000  // Record packet delay clock conversion factor
#define FPGA_CLK_PERIOD_IN_NANO_SEC  8  // Record packet delay clock period in ns
#define VERSION_BITS_DECODE  0x7F  // Version bits decode
#define VERSION_NUM_OF_BITS  7  // Number of bits required for version
#define PLAYBACK_BIT_DECODE  0x4000  // Playback FPGA bitfile identifier bit

enum CMD {
    RESET_FPGA_CMD_CODE = 0x0100,
    RESET_AR_DEV_CMD_CODE = 0x0200,
    CONFIG_FPGA_GEN_CMD_CODE = 0x0300,
    CONFIG_EEPROM_CMD_CODE = 0x0400,
    RECORD_START_CMD_CODE = 0x0500,
    RECORD_STOP_CMD_CODE = 0x0600,
    PLAYBACK_START_CMD_CODE = 0x0700,
    PLAYBACK_STOP_CMD_CODE = 0x0800,
    SYSTEM_CONNECT_CMD_CODE = 0x0900,
    SYSTEM_ERROR_CMD_CODE = 0x0a00,
    CONFIG_PACKET_DATA_CMD_CODE = 0x0b00,
    CONFIG_DATA_MODE_AR_DEV_CMD_CODE = 0x0c00,
    INIT_FPGA_PLAYBACK_CMD_CODE = 0x0d00,
    READ_FPGA_VERSION_CMD_CODE = 0x0e00
};

inline std::map<uint16_t, std::string> STATUS_STR{
    {0x0000, "SUCCESS"},
    {0x0001, "FAILURE"},
    {0x0002, "UNKNOWN COMMAND"},
};

PACK(typedef struct {
    uint32_t seqNum;
    uint8_t byteCnt[6];
    uint8_t payload[packetSize_d-10];
}) packet_t;

inline std::mutex udp_mutex;
inline std::atomic<bool> udp_continue_g = true;
inline UnlockQueue<packet_t> *udp_queue_g = nullptr;
inline std::thread udp_thread_g;

inline std::vector<uint8_t> hexStringToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    if (hex.size() % 2 != 0) {
        throw std::invalid_argument("Hex string length must be even.");
    }
    bytes.reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2) {
        uint8_t byte = static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

class RadarUDP {
    radar_socket_platform::socket_handle_t config_socket;
    radar_socket_platform::socket_handle_t data_socket;

    sockaddr_in cfg_dest_addr;
    sockaddr_in cfg_recv_addr;
    sockaddr_in data_recv_addr;

    static void create_socket(radar_socket_platform::socket_handle_t& sock) {
        sock = radar_socket_platform::create_udp_socket_handle();
        if (sock == radar_socket_platform::kInvalidSocket) {
            throw std::runtime_error("Failed to create socket");
        }
    }

public:
    RadarUDP(const std::string& adc_ip,
                 const std::string& static_ip,
                 const uint16_t config_port,
                 const uint16_t data_port) {

        memset(&cfg_dest_addr, 0, sizeof(cfg_dest_addr));
        cfg_dest_addr.sin_family = AF_INET;
        cfg_dest_addr.sin_port = htons(config_port);
        if (inet_pton(AF_INET, adc_ip.c_str(), &cfg_dest_addr.sin_addr) <= 0) {
            throw std::invalid_argument("Invalid ADC IP address");
        }

        memset(&cfg_recv_addr, 0, sizeof(cfg_recv_addr));
        cfg_recv_addr.sin_family = AF_INET;
        cfg_recv_addr.sin_port = htons(config_port);
        if (inet_pton(AF_INET, static_ip.c_str(), &cfg_recv_addr.sin_addr) <= 0) {
            throw std::invalid_argument("Invalid Static IP address for config");
        }

        memset(&data_recv_addr, 0, sizeof(data_recv_addr));
        data_recv_addr.sin_family = AF_INET;
        data_recv_addr.sin_port = htons(data_port);
        if (inet_pton(AF_INET, static_ip.c_str(), &data_recv_addr.sin_addr) <= 0) {
            throw std::invalid_argument("Invalid Static IP address for data");
        }

        config_socket = radar_socket_platform::kInvalidSocket;
        data_socket = radar_socket_platform::kInvalidSocket;
        udp_connection();
    }

    ~RadarUDP() {
        radar_socket_platform::close_socket_handle(config_socket);
        radar_socket_platform::close_socket_handle(data_socket);
    }

    void udp_connection() {
        create_socket(config_socket);
        if (bind(config_socket, reinterpret_cast<sockaddr *>(&cfg_recv_addr), sizeof(cfg_recv_addr)) < 0) {
            const int bind_error = radar_socket_platform::last_socket_error_code();
            radar_socket_platform::close_socket_handle(config_socket);
            throw std::runtime_error("Config socket bind failed: " + radar_socket_platform::socket_error_message(bind_error));
        }
        std::cout << "Config socket bind success on "
                << inet_ntoa(cfg_recv_addr.sin_addr) << ":"
                << ntohs(cfg_recv_addr.sin_port) << std::endl;

        create_socket(data_socket);
        if (bind(data_socket, reinterpret_cast<sockaddr *>(&data_recv_addr), sizeof(data_recv_addr)) < 0) {
            const int bind_error = radar_socket_platform::last_socket_error_code();
            radar_socket_platform::close_socket_handle(data_socket);
            throw std::runtime_error("Data socket bind failed: " + radar_socket_platform::socket_error_message(bind_error));
        }
        std::cout << "Data socket bind success on "
                << inet_ntoa(data_recv_addr.sin_addr) << ":"
                << ntohs(data_recv_addr.sin_port) << std::endl;
    }

    std::string send_command(const CMD cmd, const std::vector<uint8_t>& body = {}, const int timeout_sec = 4) {
        uint8_t buffer[MAX_PACKET_SIZE];
        uint8_t recv_buffer[MAX_PACKET_SIZE];

        const std::vector<uint8_t> header_bytes = hexStringToBytes(CONFIG_HEADER);

        const std::vector cmd_bytes = {
            static_cast<uint8_t>((cmd >> 8) & 0xFF),
            static_cast<uint8_t>(cmd & 0xFF)
        };

        const uint16_t body_len = body.size();
        const uint16_t net_body_len = body_len;
        uint8_t length_bytes[2];
        memcpy(length_bytes, &net_body_len, 2);

        const std::vector<uint8_t> footer_bytes = hexStringToBytes(CONFIG_FOOTER);

        size_t packet_size = 0;
        memcpy(buffer + packet_size, header_bytes.data(), header_bytes.size()); packet_size += header_bytes.size();
        memcpy(buffer + packet_size, cmd_bytes.data(), cmd_bytes.size()); packet_size += cmd_bytes.size();
        memcpy(buffer + packet_size, length_bytes, 2); packet_size += 2;
        if (!body.empty()) {
            memcpy(buffer + packet_size, body.data(), body.size());
            packet_size += body.size();
        }
        memcpy(buffer + packet_size, footer_bytes.data(), footer_bytes.size()); packet_size += footer_bytes.size();

        timeval tv;
        tv.tv_sec = timeout_sec;
        tv.tv_usec = 0;
        setsockopt(config_socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));

        // std::cout << "Buffer content (" << packet_size << " bytes): ";
        // for (size_t i = 0; i < packet_size; ++i) {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0')
        //               << static_cast<unsigned int>(static_cast<unsigned char>(buffer[i])) << " ";
        // }
        // std::cout << std::dec << std::endl;

        if (const int sent_bytes = sendto(config_socket, reinterpret_cast<const char*>(buffer), static_cast<int>(packet_size), 0, reinterpret_cast<struct sockaddr *>(&cfg_dest_addr), sizeof(cfg_dest_addr)); sent_bytes < 0) {
            throw std::runtime_error("Failed to send command");
        }

        sockaddr_in sender_addr;
        radar_socket_platform::sockaddr_length_t sender_addr_len = sizeof(sender_addr);
        const int recv_bytes = radar_socket_platform::recvfrom_socket(
            config_socket,
            reinterpret_cast<char*>(recv_buffer),
            MAX_PACKET_SIZE,
            reinterpret_cast<struct sockaddr *>(&sender_addr),
            &sender_addr_len);
        if (recv_bytes < 0) {
            throw std::runtime_error("UDP recvfrom returns <0");
        }

        if (recv_bytes < 8) {
            throw std::runtime_error("Invalid response size");
        }

        uint16_t res_header, res_cmd, res_status, res_footer;
        memcpy(&res_header, recv_buffer, 2);
        memcpy(&res_cmd, recv_buffer + 2, 2);
        memcpy(&res_status, recv_buffer + 4, 2);
        memcpy(&res_footer, recv_buffer + 6, 2);

        if (res_header != HEADER_NUM || ntohs(res_cmd) != static_cast<uint16_t>(cmd) || res_footer != FOOTER_NUM) {
            std::cout << std::hex << std::showbase;
            std::cout << "header: " << res_header << ", HEADER_NUM: " << HEADER_NUM << std::endl;
            std::cout << "cmd: " << ntohs(res_cmd) << ", cmd: " << static_cast<uint16_t>(cmd) << std::endl;
            std::cout << "footer: " << res_footer << ", FOOTER_NUM: " << FOOTER_NUM << std::endl;
            throw std::runtime_error("UDP packet error");
        }

        return STATUS_STR.count(res_status) ? STATUS_STR[res_status] : "UNKNOWN STATUS";
    }

    void reset_fpga() {
        // 5a a5 01 00 00 00 aa ee
        send_command(RESET_FPGA_CMD_CODE);
    }

    void reset_radar() {
        // 5a a5 02 00 00 00 aa ee
        send_command(RESET_AR_DEV_CMD_CODE);
    }

    std::string stream_start() {
        try {
            return send_command(RECORD_START_CMD_CODE);
        } catch (const std::exception& e) {
            std::cerr << "Error in stream_start: " << e.what() << std::endl;
            return "";
        }
    }

    std::string stream_stop() {
        try {
            return send_command(RECORD_STOP_CMD_CODE);
        } catch (const std::exception& e) {
            std::cerr << "Error in stream_stop: " << e.what() << std::endl;
            return "";
        }
    }

    std::string sys_alive_check() {
        const auto res = send_command(SYSTEM_CONNECT_CMD_CODE);
        if (!res.empty() && res.size() >= 8) {
            uint16_t header, cmd, status, footer;
            memcpy(&header, &res[0], 2);
            memcpy(&cmd, &res[2], 2);
            memcpy(&status, &res[4], 2);
            memcpy(&footer, &res[6], 2);

            if (header != HEADER_NUM || cmd != SYSTEM_CONNECT_CMD_CODE || footer != FOOTER_NUM) {
                std::cerr << "UDP packet error in sys_alive_check" << std::endl;
                return "Packet Error";
            }
            return STATUS_STR[status];

        }
        std::cerr << "No response in sys_alive_check" << std::endl;
        return "No Response";
    }

    std::string read_fpga_version() {
        const auto res = send_command(READ_FPGA_VERSION_CMD_CODE);
        if (!res.empty() && res.size() >= 8) {
            uint16_t header, cmd, ver_num, footer;
            memcpy(&header, &res[0], 2);
            memcpy(&cmd, &res[2], 2);
            memcpy(&ver_num, &res[4], 2);
            memcpy(&footer, &res[6], 2);

            if (header != HEADER_NUM || cmd != READ_FPGA_VERSION_CMD_CODE || footer != FOOTER_NUM) {
                std::cerr << "UDP packet error in read_fpga_version" << std::endl;
                return "Packet Error";
            }

            const uint16_t major_version = ver_num & VERSION_BITS_DECODE;
            const uint16_t minor_version = (ver_num >> VERSION_NUM_OF_BITS) & VERSION_BITS_DECODE;
            std::ostringstream ret_str;
            ret_str << "FPGA Version: " << major_version << "." << minor_version << " ";
            ret_str << ((ver_num & PLAYBACK_BIT_DECODE) ? "[Playback]" : "[Record]");
            return ret_str.str();
        }
        std::cerr << "No response in read_fpga_version" << std::endl;
        return "No Response";
    }

    std::string config_fpga(const std::string& filepath) {
        std::map<std::string, uint8_t> config_log_mode{{"raw", 0x01}, {"multi", 0x02}};
        std::map<int, uint8_t> config_lvds_mode{{1, 0x01}, {2, 0x02}};
        std::map<std::string, uint8_t> config_transfer_mode{{"LVDSCapture", 0x01}, {"playback", 0x02}};
        std::map<std::string, uint8_t> config_capture_mode{{"SDCardStorage", 0x01}, {"ethernetStream", 0x02}};
        std::map<int, uint8_t> config_format_mode{{1, 0x01}, {2, 0x02}, {3, 0x03}};
        uint8_t config_timer = 0x1e; // not supported

        std::ifstream ifs(filepath);
        if (!ifs.is_open()) {
            std::cerr << "Failed to open dca config file." << std::endl;
            return "File Error";
        }

        std::map<std::string, std::string> config_values;
        std::string line;

        while (std::getline(ifs, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string value;

            if (std::string key; std::getline(iss, key, '=') && std::getline(iss, value)) {
                config_values[key] = value;
            }
        }

        std::vector payload = {
            config_log_mode.at(config_values.at("dataLoggingMode")),
            config_lvds_mode.at(std::stoi(config_values.at("lvdsMode"))),
            config_transfer_mode.at(config_values.at("dataTransferMode")),
            config_capture_mode.at(config_values.at("dataCaptureMode")),
            config_format_mode.at(std::stoi(config_values.at("dataFormatMode"))),
            config_timer
        };

        // 5a a5 03 00 06 00 01 01 01 02 03 1e aa ee
        auto res = send_command(CONFIG_FPGA_GEN_CMD_CODE, payload);
        if (res != "SUCCESS") {
            std::cerr << "UDP packet error in config_fpga" << std::endl;
            return "Packet Error";
        }

        constexpr uint16_t bytes_per_packet = MAX_BYTES_PER_PACKET;
        int packetDelay_us = std::stoi(config_values.at("packetDelay_us"));
        uint16_t record_delay = static_cast<uint16_t>(packetDelay_us * FPGA_CLK_CONVERSION_FACTOR / FPGA_CLK_PERIOD_IN_NANO_SEC);
        constexpr uint16_t reserved = 0x0000;

        std::vector<uint8_t> payload_record(6);
        memcpy(&payload[0], &bytes_per_packet, 2);
        memcpy(&payload[2], &record_delay, 2);
        memcpy(&payload[4], &reserved, 2);
        res = send_command(CONFIG_PACKET_DATA_CMD_CODE, payload_record);
        if (res != "SUCCESS") {
            std::cerr << "UDP packet error in config_fpga" << std::endl;
            return "Packet Error";
        }

        return "config success";
    }

     size_t udp_read_thread_init(const int maxPacketNum) const {
        if (udp_queue_g != nullptr) {
            delete udp_queue_g;
        }
        udp_queue_g = new UnlockQueue<packet_t>(maxPacketNum);
        clear_udp_buffer(data_socket);
        udp_read_thread_start(data_socket);
        return udp_queue_g->size();
    }

    static void udp_read_thread_start(radar_socket_platform::socket_handle_t sock_fd) {
        udp_continue_g = true;
        udp_thread_g = std::thread(_udp_read_thread, sock_fd);
    }

    static std::vector<uint8_t> udp_read_thread_get_packets(const uint32_t packetNum, const int timeout_ms, const bool sort, uint32_t &firstSeqNum, uint32_t &lastSeqNum) {
        std::vector<uint8_t> result(packetNum * packetSize_d);
        const auto buf_ptr = reinterpret_cast<packet_t*>(result.data());

        const uint32_t ret = udp_queue_g->Get_wait(buf_ptr, packetNum, timeout_ms);
        if (ret < packetNum)
            std::cout << "[udp] get " << ret << ", expected " << packetNum << std::endl;

        if(sort) return packet_sort(result, ret, firstSeqNum, lastSeqNum);

        return result;
    }

    static void _udp_read_thread(const radar_socket_platform::socket_handle_t sock_fd){
        udp_mutex.lock();

        const auto sockfd = sock_fd;
        if (!radar_socket_platform::set_socket_nonblocking(sockfd))
            throw std::runtime_error("[udp] error calling socket nonblocking setup");

        sockaddr_in src;
        radar_socket_platform::sockaddr_length_t src_len = sizeof(src);
        memset(&src, 0, sizeof(src));

        packet_t buffer;

        while (udp_continue_g) {
            // Try to receive data in non-blocking mode
            if (const int n = radar_socket_platform::recvfrom_socket(sockfd, reinterpret_cast<char*>(&buffer), packetSize_d, reinterpret_cast<sockaddr *>(&src), &src_len); n > 0) { // Data received
                while (udp_queue_g->Put(&buffer, 1) == 0);
            }else{ // No data available
                // std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
        udp_mutex.unlock();
    }

    static void udp_read_thread_stop() {
        if (udp_continue_g) {
            udp_continue_g = false;
            if (udp_thread_g.joinable()) {
                udp_thread_g.join();
            }
            if (udp_queue_g) {
                delete udp_queue_g;
                udp_queue_g = nullptr;
            }
        }
    }

    static std::vector<uint8_t> packet_sort(const std::vector<uint8_t>& buf, const uint32_t packetNum, uint32_t &firstSeqNum, uint32_t &lastSeqNum) {
        if (packetNum == 0 || buf.size() < packetNum * packetSize_d) {
            return {};
        }

        const uint8_t* buf_ptr = buf.data();

        uint32_t minSeqNum = std::numeric_limits<uint32_t>::max();
        for (uint32_t i = 0; i < packetNum; i++) {
            if (const uint32_t seqNum = *reinterpret_cast<const uint32_t *>(&buf_ptr[i * packetSize_d]); seqNum < minSeqNum) {
                minSeqNum = seqNum;
            }
        }

        std::vector<uint8_t> result(packetNum * packetSize_d, 0);
        uint8_t* sort_buf_ptr = result.data();

        uint32_t receivedPacketNum = 0;
        for (uint32_t i = 0; i < packetNum; i++) {
            const uint32_t seqNum = *reinterpret_cast<const uint32_t *>(&buf_ptr[i * packetSize_d]);
            const uint32_t idx = seqNum - minSeqNum;

            if (idx >= packetNum) {
                continue;
            }

            memcpy(&sort_buf_ptr[idx * packetSize_d], &buf_ptr[i * packetSize_d], packetSize_d);
            receivedPacketNum++;
        }

        firstSeqNum = *reinterpret_cast<const uint32_t *>(sort_buf_ptr);
        lastSeqNum = *reinterpret_cast<const uint32_t *>(&sort_buf_ptr[(packetNum - 1) * packetSize_d]);

        return result;
    }

    static void clear_udp_buffer(const radar_socket_platform::socket_handle_t sock_fd) {
        char temp_buffer[2048];

        const auto sockfd = sock_fd;
        if (!radar_socket_platform::set_socket_nonblocking(sockfd))
            throw std::runtime_error("[udp] error calling socket nonblocking setup");

        while (true) {
            if (const int n = radar_socket_platform::recvfrom_socket(sockfd, temp_buffer, static_cast<int>(sizeof(temp_buffer)), nullptr, nullptr); n < 0) {
                if (radar_socket_platform::socket_would_block()) {
                    break;
                }
                std::cerr << "recvfrom error: " << radar_socket_platform::last_socket_error_message() << std::endl;
                break;
            }
        }
        std::cout << "[udp] udp buffer is cleared" << std::endl;
    }
};
