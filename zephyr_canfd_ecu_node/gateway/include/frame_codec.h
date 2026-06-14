#ifndef FRAME_CODEC_H
#define FRAME_CODEC_H

#include <stdint.h>
#include <stddef.h>
#include <string>

#define TELEMETRY_CAN_ID 0x100
#define COMMAND_CAN_ID   0x200
#define RESPONSE_CAN_ID  0x201

#pragma pack(push, 1)
struct TelemetryData {
    double rpm;
    double coolant_temp;
    double oil_pressure;
    double battery_voltage;
    double fuel_level;
    double vehicle_speed;
    double ambient_temp;
    uint32_t fault_bitmap;
    uint16_t sequence_counter;
    uint32_t uptime;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawTelemetryFrame {
    uint16_t rpm;
    int16_t coolant_temp;
    uint16_t oil_pressure;
    uint16_t battery_voltage;
    uint16_t fuel_level;
    uint16_t vehicle_speed;
    int16_t ambient_temp;
    uint32_t fault_bitmap;
    uint16_t sequence_counter;
    uint32_t uptime;
    uint8_t reserved[24];
};
#pragma pack(pop)

static_assert(sizeof(RawTelemetryFrame) == 48, "RawTelemetryFrame must be exactly 48 bytes");

#pragma pack(push, 1)
struct RawCommandFrame {
    uint8_t command_id;
    uint8_t param_u8;
    uint8_t reserved[6];
};
#pragma pack(pop)

static_assert(sizeof(RawCommandFrame) == 8, "RawCommandFrame must be exactly 8 bytes");

#pragma pack(push, 1)
struct RawResponseFrame {
    uint8_t command_id;
    uint8_t status;
    uint8_t reserved[6];
};
#pragma pack(pop)

static_assert(sizeof(RawResponseFrame) == 8, "RawResponseFrame must be exactly 8 bytes");

bool decode_telemetry(const uint8_t* data, size_t len, TelemetryData& decoded);
void encode_command(uint8_t command_id, uint8_t param_u8, RawCommandFrame& frame);
bool decode_response(const uint8_t* data, size_t len, uint8_t& command_id, uint8_t& status);

#endif // FRAME_CODEC_H
