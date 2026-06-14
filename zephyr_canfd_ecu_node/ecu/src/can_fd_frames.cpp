#include "can_fd_frames.h"
#include <string.h>

static inline uint16_t swap16(uint16_t val) {
    return (val << 8) | (val >> 8);
}

static inline uint32_t swap32(uint32_t val) {
    return ((val >> 24) & 0xff) |
           ((val << 8) & 0xff0000) |
           ((val >> 8) & 0xff00) |
           ((val << 24) & 0xff000000);
}

static inline bool is_little_endian() {
    uint16_t number = 0x1;
    char *numPtr = (char*)&number;
    return (numPtr[0] == 1);
}

static inline uint16_t host_to_be16(uint16_t val) {
    return is_little_endian() ? swap16(val) : val;
}

static inline int16_t host_to_be16_signed(int16_t val) {
    return (int16_t)host_to_be16((uint16_t)val);
}

static inline uint32_t host_to_be32(uint32_t val) {
    return is_little_endian() ? swap32(val) : val;
}

void pack_telemetry(const PhysicsState& state, uint32_t fault_bitmap, uint16_t seq, uint32_t uptime, TelemetryFrame& frame) {
    frame.rpm = host_to_be16(static_cast<uint16_t>(state.rpm));
    frame.coolant_temp = host_to_be16_signed(static_cast<int16_t>(state.coolant_temp * 256.0));
    frame.oil_pressure = host_to_be16(static_cast<uint16_t>(state.oil_pressure));
    frame.battery_voltage = host_to_be16(static_cast<uint16_t>(state.battery_voltage * 100.0));
    frame.fuel_level = host_to_be16(static_cast<uint16_t>(state.fuel_level * 10.0));
    frame.vehicle_speed = host_to_be16(static_cast<uint16_t>(state.vehicle_speed * 10.0));
    frame.ambient_temp = host_to_be16_signed(static_cast<int16_t>(state.ambient_temp * 256.0));
    frame.fault_bitmap = host_to_be32(fault_bitmap);
    frame.sequence_counter = host_to_be16(seq);
    frame.uptime = host_to_be32(uptime);
    memset(frame.reserved, 0, sizeof(frame.reserved));
}

bool unpack_command(const uint8_t* data, size_t len, CommandFrame& cmd) {
    if (len < sizeof(CommandFrame)) {
        return false;
    }
    memcpy(&cmd, data, sizeof(CommandFrame));
    return true;
}

void pack_response(uint8_t command_id, uint8_t status, ResponseFrame& resp) {
    resp.command_id = command_id;
    resp.status = status;
    memset(resp.reserved, 0, sizeof(resp.reserved));
}
