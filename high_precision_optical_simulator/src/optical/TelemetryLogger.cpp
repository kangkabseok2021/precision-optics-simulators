#include "TelemetryLogger.h"
#include <chrono>
#include <fstream>

TelemetryLogger::TelemetryLogger(const std::string& csv_path, size_t capacity)
    : buf_(capacity)
{
    writer_ = std::thread(&TelemetryLogger::writerLoop, this, csv_path);
}

TelemetryLogger::~TelemetryLogger() { stop(); }

// SPSC push: tail_ written only by producer; head_ read to check fullness.
bool TelemetryLogger::push(const Frame& f) {
    size_t t    = tail_.load(std::memory_order_relaxed);
    size_t next = (t + 1) % buf_.size();
    if (next == head_.load(std::memory_order_acquire)) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    buf_[t] = f;
    tail_.store(next, std::memory_order_release);
    return true;
}

void TelemetryLogger::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (writer_.joinable()) writer_.join();
}

// SPSC consumer: head_ written only here; tail_ read to detect new frames.
void TelemetryLogger::writerLoop(const std::string& path) {
    std::ofstream file(path);
    file << "ts_us,x_um,y_um,z_um,spindle_rpm,deviation_um,event_code\n";

    for (;;) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t t = tail_.load(std::memory_order_acquire);

        if (h == t) {
            if (!running_.load(std::memory_order_relaxed)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        while (h != t) {
            const Frame& fr = buf_[h];
            file << fr.ts_us     << ','
                 << fr.x_um      << ','
                 << fr.y_um      << ','
                 << fr.z_um      << ','
                 << fr.spindle_rpm   << ','
                 << fr.deviation_um  << ','
                 << fr.event_code    << '\n';
            h = (h + 1) % buf_.size();
            head_.store(h, std::memory_order_release);
        }
        file.flush();
    }
}
