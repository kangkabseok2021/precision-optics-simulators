#include "frame_codec.h"
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

static inline uint16_t be16_to_host(uint16_t val) {
    return is_little_endian() ? swap16(val) : val;
}

static inline int16_t be16_to_host_signed(int16_t val) {
    return (int16_t)be16_to_host((uint16_t)val);
}

static inline uint32_t be32_to_host(uint32_t val) {
    return is_little_endian() ? swap32(val) : val;
}

static inline uint16_t host_to_be16(uint16_t val) {
    return is_little_endian() ? swap16(val) : val;
}

bool decode_telemetry(const uint8_t* data, size_t len, TelemetryData& decoded) {
    if (len < sizeof(RawTelemetryFrame)) {
        return false;
    }
    RawTelemetryFrame raw;
    memcpy(&raw, data, sizeof(RawTelemetryFrame));

    decoded.rpm = static_cast<double>(be16_to_host(raw.rpm));
    decoded.coolant_temp = static_cast<double>(be16_to_host_signed(raw.coolant_temp)) / 256.0;
    decoded.oil_pressure = static_cast<double>(be16_to_host(raw.oil_pressure));
    decoded.battery_voltage = static_cast<double>(be16_to_host(raw.battery_voltage)) / 100.0;
    decoded.fuel_level = static_cast<double>(be16_to_host(raw.fuel_level)) / 10.0;
    decoded.vehicle_speed = static_cast<double>(be16_to_host(raw.vehicle_speed)) / 10.0;
    decoded.ambient_temp = static_cast<double>(be16_to_host_signed(raw.ambient_temp)) / 256.0;
    decoded.fault_bitmap = be32_to_host(raw.fault_bitmap);
    decoded.sequence_counter = be16_to_host(raw.sequence_counter);
    decoded.uptime = be32_to_host(raw.uptime);

    return true;
}

void encode_command(uint8_t command_id, uint8_t param_u8, RawCommandFrame& frame) {
    frame.command_id = command_id;
    frame.param_u8 = param_u8;
    memset(frame.reserved, 0, sizeof(frame.reserved));
}

bool decode_response(const uint8_t* data, size_t len, uint8_t& command_id, uint8_t& status) {
    if (len < sizeof(RawResponseFrame)) {
        return false;
    }
    RawResponseFrame raw;
    memcpy(&raw, data, sizeof(RawResponseFrame));
    command_id = raw.command_id;
    status = raw.status;
    return true;
}
