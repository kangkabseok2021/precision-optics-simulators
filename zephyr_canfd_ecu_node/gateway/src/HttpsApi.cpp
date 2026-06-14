#include "HttpsApi.h"
#include <nlohmann/json.hpp>
#include <iostream>

using json = nlohmann::json;

static json telemetry_to_json(const TelemetryData& data) {
    return json{
        {"rpm", data.rpm},
        {"coolant_temp", data.coolant_temp},
        {"oil_pressure", data.oil_pressure},
        {"battery_voltage", data.battery_voltage},
        {"fuel_level", data.fuel_level},
        {"vehicle_speed", data.vehicle_speed},
        {"ambient_temp", data.ambient_temp},
        {"fault_bitmap", data.fault_bitmap},
        {"sequence_counter", data.sequence_counter},
        {"uptime", data.uptime}
    };
}

HttpsApi::HttpsApi(const std::string& cert_path, const std::string& key_path, int port)
    : svr_(cert_path.c_str(), key_path.c_str()), port_(port), has_data_(false) {
    
    svr_.Get("/api/telemetry/latest", [this](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_data_) {
            res.status = 404;
            res.set_content("{\"error\": \"No telemetry data available yet\"}", "application/json");
            return;
        }
        res.set_content(telemetry_to_json(latest_data_).dump(), "application/json");
    });

    svr_.Get("/api/telemetry/history", [this](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        json history_arr = json::array();
        for (const auto& item : history_) {
            history_arr.push_back(telemetry_to_json(item));
        }
        res.set_content(history_arr.dump(), "application/json");
    });
}

HttpsApi::~HttpsApi() {
    stop();
}

bool HttpsApi::start() {
    server_thread_ = std::thread(&HttpsApi::listen_loop, this);
    return true;
}

void HttpsApi::stop() {
    svr_.stop();
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}

void HttpsApi::update_telemetry(const TelemetryData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_ = data;
    has_data_ = true;
    
    history_.push_back(data);
    if (history_.size() > max_history_size_) {
        history_.pop_front();
    }
}

void HttpsApi::listen_loop() {
    std::cout << "Starting HTTPS server on port " << port_ << "..." << std::endl;
    if (!svr_.listen("0.0.0.0", port_)) {
        std::cerr << "Failed to start HTTPS server on port " << port_ << std::endl;
    }
}
