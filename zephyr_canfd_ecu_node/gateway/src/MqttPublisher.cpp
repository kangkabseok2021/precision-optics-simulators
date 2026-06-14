#include "MqttPublisher.h"
#include <nlohmann/json.hpp>
#include <iostream>

using json = nlohmann::json;

MqttPublisher::MqttPublisher(const std::string& broker_address, const std::string& client_id)
    : client_(broker_address, client_id) {
    client_.set_callback(*this);
}

MqttPublisher::~MqttPublisher() {
    disconnect();
}

bool MqttPublisher::connect() {
	try {
		mqtt::connect_options connOpts;
		connOpts.set_keep_alive_interval(20);
		connOpts.set_clean_session(true);

		client_.connect(connOpts)->wait();
		client_.subscribe("command/ecu/#", 1)->wait();
		std::cout << "Connected to MQTT broker and subscribed to command/ecu/#" << std::endl;
		return true;
	} catch (const mqtt::exception& exc) {
		std::cerr << "MQTT connection failed: " << exc.what() << std::endl;
		return false;
	}
}

void MqttPublisher::disconnect() {
	if (client_.is_connected()) {
		try {
			client_.disconnect()->wait();
		} catch (...) {}
	}
}

void MqttPublisher::publish_telemetry(const TelemetryData& data) {
	if (!client_.is_connected()) return;

	try {
		json payload = {
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

		std::string payload_str = payload.dump();
		client_.publish("telemetry/ecu/all", payload_str, 1, false);

		client_.publish("telemetry/ecu/rpm", std::to_string(data.rpm), 1, false);
		client_.publish("telemetry/ecu/coolant_temp", std::to_string(data.coolant_temp), 1, false);
		client_.publish("telemetry/ecu/oil_pressure", std::to_string(data.oil_pressure), 1, false);
		client_.publish("telemetry/ecu/battery_voltage", std::to_string(data.battery_voltage), 1, false);
		client_.publish("telemetry/ecu/fuel_level", std::to_string(data.fuel_level), 1, false);
		client_.publish("telemetry/ecu/vehicle_speed", std::to_string(data.vehicle_speed), 1, false);
		client_.publish("telemetry/ecu/fault_bitmap", std::to_string(data.fault_bitmap), 1, false);

	} catch (const mqtt::exception& exc) {
		std::cerr << "Failed to publish MQTT telemetry: " << exc.what() << std::endl;
	}
}

void MqttPublisher::register_command_callback(CommandCallback cb) {
	command_callback_ = cb;
}

void MqttPublisher::connection_lost(const std::string& cause) {
	std::cout << "MQTT Connection lost: " << cause << std::endl;
}

void MqttPublisher::message_arrived(mqtt::const_message_ptr msg) {
	std::string topic = msg->get_topic();
	std::string payload = msg->to_string();

	std::cout << "MQTT Message arrived on topic: " << topic << " payload: " << payload << std::endl;

	if (command_callback_) {
		try {
			auto parsed = json::parse(payload);
			uint8_t cmd_id = 0;
			uint8_t param = 0;

			if (topic.ends_with("set_mode")) {
				cmd_id = 0x01;
				param = parsed.value("mode", 0);
			} else if (topic.ends_with("set_throttle")) {
				cmd_id = 0x02;
				param = parsed.value("throttle", 0);
			} else if (topic.ends_with("request_diag")) {
				cmd_id = 0x03;
			} else if (topic.ends_with("clear_faults")) {
				cmd_id = 0x04;
			}

			if (cmd_id != 0) {
				command_callback_(cmd_id, param);
			}
		} catch (const std::exception& e) {
			std::cerr << "Failed to parse MQTT command JSON: " << e.what() << std::endl;
		}
	}
}

void MqttPublisher::delivery_complete(mqtt::delivery_token_ptr token) {
}
