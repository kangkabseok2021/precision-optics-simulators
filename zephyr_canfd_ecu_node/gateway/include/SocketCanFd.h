#ifndef SOCKET_CAN_FD_H
#define SOCKET_CAN_FD_H

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <vector>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#else
#define CANFD_MTU 72
#define CAN_RAW_FD_FRAMES 5
#define SOL_CAN_RAW 101
struct canfd_frame {
    uint32_t can_id;
    uint8_t len;
    uint8_t flags;
    uint8_t __res0;
    uint8_t __res1;
    uint8_t data[64];
};
#endif

class SocketCanFd {
public:
    SocketCanFd();
    ~SocketCanFd();

    bool open_bus(const std::string& interface_name);
    void close_bus();

    bool write_frame(uint32_t can_id, const uint8_t* data, uint8_t len, bool fd_mode = false);
    
    using FrameCallback = std::function<void(uint32_t can_id, const uint8_t* data, uint8_t len)>;
    void register_callback(FrameCallback cb);

    bool is_open() const;

private:
    void receive_loop();

    int socket_fd_;
    std::string interface_;
    std::atomic<bool> running_;
    std::thread rx_thread_;
    FrameCallback callback_;
    std::mutex callback_mutex_;
};

#endif // SOCKET_CAN_FD_H
