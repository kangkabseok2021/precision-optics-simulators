#ifndef HTTPS_API_H
#define HTTPS_API_H

#define CPPHTTPLIB_OPENSSL_SUPPORT
#include <httplib.h>
#include <mutex>
#include <deque>
#include <string>
#include <thread>
#include "frame_codec.h"

class HttpsApi {
public:
    HttpsApi(const std::string& cert_path, const std::string& key_path, int port);
    ~HttpsApi();

    bool start();
    void stop();

    void update_telemetry(const TelemetryData& data);

private:
    void listen_loop();

    httplib::SSLServer svr_;
    int port_;
    std::thread server_thread_;
    
    std::mutex data_mutex_;
    TelemetryData latest_data_;
    bool has_data_;
    std::deque<TelemetryData> history_;
    const size_t max_history_size_ = 50;
};

#endif // HTTPS_API_H
