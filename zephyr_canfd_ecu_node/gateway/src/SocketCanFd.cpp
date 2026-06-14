#include "SocketCanFd.h"
#include <iostream>

#ifdef __linux__
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <string.h>
#endif

SocketCanFd::SocketCanFd() : socket_fd_(-1), running_(false) {}

SocketCanFd::~SocketCanFd() {
    close_bus();
}

bool SocketCanFd::open_bus(const std::string& interface_name) {
    interface_ = interface_name;
#ifdef __linux__
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to open CAN socket" << std::endl;
        return false;
    }

    int enable_fd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        std::cerr << "Failed to enable CAN-FD support" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index for " << interface_name << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind CAN socket" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    running_ = true;
    rx_thread_ = std::thread(&SocketCanFd::receive_loop, this);
    return true;
#else
    std::cout << "[MOCK] Opened virtual CAN-FD bus on interface: " << interface_name << std::endl;
    socket_fd_ = 999;
    running_ = true;
    rx_thread_ = std::thread(&SocketCanFd::receive_loop, this);
    return true;
#endif
}

void SocketCanFd::close_bus() {
    running_ = false;
    if (socket_fd_ >= 0) {
#ifdef __linux__
        close(socket_fd_);
#endif
        socket_fd_ = -1;
    }
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

bool SocketCanFd::write_frame(uint32_t can_id, const uint8_t* data, uint8_t len, bool fd_mode) {
    if (socket_fd_ < 0) return false;

#ifdef __linux__
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.len = len;
    if (fd_mode) {
        frame.flags = 0x01 | 0x02; 
    }
    memcpy(frame.data, data, len);

    size_t write_size = fd_mode ? CANFD_MTU : CAN_MTU;
    ssize_t bytes_written = write(socket_fd_, &frame, write_size);
    return bytes_written == static_cast<ssize_t>(write_size);
#else
    std::cout << "[MOCK] Write CAN Frame: ID=0x" << std::hex << can_id 
              << std::dec << " Len=" << (int)len << " FD=" << fd_mode << std::endl;
    return true;
#endif
}

void SocketCanFd::register_callback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = cb;
}

bool SocketCanFd::is_open() const {
    return socket_fd_ >= 0;
}

void SocketCanFd::receive_loop() {
#ifdef __linux__
    struct canfd_frame frame;
    while (running_) {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0) {
            if (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }

        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (callback_) {
            callback_(frame.can_id, frame.data, frame.len);
        }
    }
#else
    uint16_t seq = 0;
    uint32_t uptime = 0;
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (callback_ && running_) {
            uint8_t mock_data[48];
            memset(mock_data, 0, sizeof(mock_data));
            
            mock_data[0] = 0x09;
            mock_data[1] = 0x60;
            mock_data[2] = 0x5A;
            mock_data[3] = 0x00;
            mock_data[4] = 0x01;
            mock_data[5] = 0x2C;
            mock_data[6] = 0x05;
            mock_data[7] = 0x64;
            
            mock_data[18] = (seq >> 8) & 0xff;
            mock_data[19] = seq & 0xff;
            seq++;
            
            uptime++;
            mock_data[20] = (uptime >> 24) & 0xff;
            mock_data[21] = (uptime >> 16) & 0xff;
            mock_data[22] = (uptime >> 8) & 0xff;
            mock_data[23] = uptime & 0xff;

            callback_(0x100, mock_data, 48);
        }
    }
#endif
}
