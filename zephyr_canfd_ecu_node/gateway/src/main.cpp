#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <atomic>

#include "SocketCanFd.h"
#include "frame_codec.h"
#include "MqttPublisher.h"
#include "HttpsApi.h"

std::atomic<bool> keep_running(true);

void signal_handler(int sig) {
    std::cout << "\nReceived signal " << sig << ", shutting down..." << std::endl;
    keep_running = false;
}

int main(int argc, char* argv[]) {
    // Register signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Default configuration values
    std::string can_interface = "vcan0";
    std::string mqtt_broker = "tcp://localhost:1883";
    std::string mqtt_client_id = "ecu_gateway";
    std::string cert_path = "cert.pem";
    std::string key_path = "key.pem";
    int https_port = 8443;

    // Basic CLI arg parsing
    if (argc > 1) can_interface = argv[1];
    if (argc > 2) mqtt_broker = argv[2];
    if (argc > 3) https_port = std::stoi(argv[3]);
    if (argc > 4) cert_path = argv[4];
    if (argc > 5) key_path = argv[5];

    std::cout << "Initializing ECU CAN-FD Gateway..." << std::endl;
    std::cout << "CAN Interface: " << can_interface << std::endl;
    std::cout << "MQTT Broker:   " << mqtt_broker << std::endl;
    std::cout << "HTTPS Port:    " << https_port << std::endl;

    // 1. Initialize CAN-FD Bus Driver
    SocketCanFd can_bus;

    // 2. Initialize MQTT Publisher
    MqttPublisher mqtt_pub(mqtt_broker, mqtt_client_id);

    // 3. Initialize HTTPS REST API
    // Ensure cert/key files exist or use defaults
    HttpsApi https_api(cert_path, key_path, https_port);

    // 4. Hook up CAN RX callback
    can_bus.register_callback([&](uint32_t can_id, const uint8_t* data, uint8_t len) {
        if (can_id == TELEMETRY_CAN_ID) {
            TelemetryData decoded;
            if (decode_telemetry(data, len, decoded)) {
                // Update HTTPS local cache & history
                https_api.update_telemetry(decoded);
                // Publish to MQTT
                mqtt_pub.publish_telemetry(decoded);
            }
        } else if (can_id == RESPONSE_CAN_ID) {
            uint8_t cmd_id = 0;
            uint8_t status = 0;
            if (decode_response(data, len, cmd_id, status)) {
                std::cout << "Received ECU ACK for command 0x" 
                          << std::hex << (int)cmd_id 
                          << " with status " << (int)status << std::dec << std::endl;
            }
        }
    });

    // 5. Hook up MQTT command callback
    mqtt_pub.register_command_callback([&](uint8_t command_id, uint8_t param_u8) {
        RawCommandFrame frame;
        encode_command(command_id, param_u8, frame);
        
        std::cout << "Forwarding MQTT command (ID 0x" << std::hex << (int)command_id 
                  << ", param=" << (int)param_u8 << ") onto CAN bus as 0x200 frame" << std::dec << std::endl;
                  
        // Write as classic CAN command (DLC 8, no FD BRS flags needed)
        can_bus.write_frame(COMMAND_CAN_ID, reinterpret_cast<const uint8_t*>(&frame), sizeof(RawCommandFrame), false);
    });

    // 6. Connect to Bus and Broker
    if (!can_bus.open_bus(can_interface)) {
        std::cerr << "CRITICAL: Could not open CAN interface " << can_interface << std::endl;
        return 1;
    }

    // Try connecting to MQTT; print warning if offline (e.g. during local tests) but continue
    if (!mqtt_pub.connect()) {
        std::cerr << "WARNING: MQTT Broker offline, running in local-only mode" << std::endl;
    }

    // Start HTTPS REST API
    https_api.start();

    std::cout << "Gateway successfully started. Press Ctrl+C to terminate." << std::endl;

    // Main thread event/alive loop
    while (keep_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Stopping gateway services..." << std::endl;
    https_api.stop();
    mqtt_pub.disconnect();
    can_bus.close_bus();
    std::cout << "Gateway shutdown complete." << std::endl;

    return 0;
}
