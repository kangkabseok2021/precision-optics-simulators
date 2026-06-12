#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

// Lock-free SPSC ring-buffer telemetry logger.
// Producer (control loop): push() — never blocks.
// Consumer (writer thread): drains buffer, writes CSV every 10 ms.
// stop() signals the writer to flush remaining frames then exit.
class TelemetryLogger {
public:
    struct Frame {
        int64_t ts_us;
        double  x_um, y_um, z_um;
        double  spindle_rpm;
        double  deviation_um;
        int     event_code;   // 0=normal, 1=warn, 2=fault
    };

    explicit TelemetryLogger(const std::string& csv_path, size_t capacity = 1024);
    ~TelemetryLogger();

    // Non-blocking push — returns false (and increments dropped()) if buffer full.
    bool push(const Frame& f);

    // Signal writer thread to flush remaining frames and exit; blocks until done.
    void stop();

    size_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    std::vector<Frame>    buf_;
    std::atomic<size_t>   head_{0};
    std::atomic<size_t>   tail_{0};
    std::atomic<size_t>   dropped_{0};
    std::atomic<bool>     running_{true};
    std::thread           writer_;

    void writerLoop(const std::string& path);
};
