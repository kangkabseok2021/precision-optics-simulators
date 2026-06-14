#ifndef CAN_FD_FRAMES_H
#define CAN_FD_FRAMES_H

#include <stdint.h>
#include <stddef.h>
#include "physics_model.h"

#define TELEMETRY_CAN_ID 0x100
#define COMMAND_CAN_ID   0x200
#define RESPONSE_CAN_ID  0x201

#pragma pack(push, 1)
struct TelemetryFrame {
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

static_assert(sizeof(TelemetryFrame) == 48, "TelemetryFrame size must be exactly 48 bytes");

#pragma pack(push, 1)
struct CommandFrame {
    uint8_t command_id;
    uint8_t param_u8;
    uint8_t reserved[6];
};
#pragma pack(pop)

static_assert(sizeof(CommandFrame) == 8, "CommandFrame size must be exactly 8 bytes");

#pragma pack(push, 1)
struct ResponseFrame {
    uint8_t command_id;
    uint8_t status;
    uint8_t reserved[6];
};
#pragma pack(pop)

static_assert(sizeof(ResponseFrame) == 8, "ResponseFrame size must be exactly 8 bytes");

void pack_telemetry(const PhysicsState& state, uint32_t fault_bitmap, uint16_t seq, uint32_t uptime, TelemetryFrame& frame);
bool unpack_command(const uint8_t* data, size_t len, CommandFrame& cmd);
void pack_response(uint8_t command_id, uint8_t status, ResponseFrame& resp);

#endif // CAN_FD_FRAMES_H
