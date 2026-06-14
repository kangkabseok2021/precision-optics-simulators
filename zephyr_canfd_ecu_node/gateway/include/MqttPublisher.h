#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <string>
#include <memory>
#include <functional>
#include <mqtt/async_client.h>
#include "frame_codec.h"

class MqttPublisher : public virtual mqtt::callback {
public:
    using CommandCallback = std::function<void(uint8_t command_id, uint8_t param_u8)>;

    MqttPublisher(const std::string& broker_address, const std::string& client_id);
    ~MqttPublisher();

    bool connect();
    void disconnect();

    void publish_telemetry(const TelemetryData& data);
    void register_command_callback(CommandCallback cb);

    void connection_lost(const std::string& cause) override;
    void message_arrived(mqtt::const_message_ptr msg) override;
    void delivery_complete(mqtt::delivery_token_ptr token) override;

private:
    mqtt::async_client client_;
    CommandCallback command_callback_;
};

#endif // MQTT_PUBLISHER_H
