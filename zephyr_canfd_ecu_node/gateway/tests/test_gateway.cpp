#include <gtest/gtest.h>
#include <string.h>

#include "frame_codec.h"
#include "HttpsApi.h"

TEST(GatewayCodecTests, TestDecodeTelemetry) {
    // 48 bytes telemetry frame
    uint8_t frame_bytes[48];
    memset(frame_bytes, 0, sizeof(frame_bytes));

    // RPM = 3000 (0x0BB8) -> bytes 0-1
    frame_bytes[0] = 0x0B;
    frame_bytes[1] = 0xB8;

    // Coolant Temp = 85.5 °C (85.5 * 256 = 21888 = 0x5580) -> bytes 2-3
    frame_bytes[2] = 0x55;
    frame_bytes[3] = 0x80;

    // Oil Pressure = 450 kPa (0x01C2) -> bytes 4-5
    frame_bytes[4] = 0x01;
    frame_bytes[5] = 0xC2;

    // Battery Voltage = 13.84 V (13.84 * 100 = 1384 = 0x0568) -> bytes 6-7
    frame_bytes[6] = 0x05;
    frame_bytes[7] = 0x68;

    // Fuel level = 75.2 % (75.2 * 10 = 752 = 0x02F0) -> bytes 8-9
    frame_bytes[8] = 0x02;
    frame_bytes[9] = 0xF0;

    // Speed = 100.5 km/h (100.5 * 10 = 1005 = 0x03ED) -> bytes 10-11
    frame_bytes[10] = 0x03;
    frame_bytes[11] = 0xED;

    // Ambient Temp = 22.0 °C (22.0 * 256 = 5632 = 0x1600) -> bytes 12-13
    frame_bytes[12] = 0x16;
    frame_bytes[13] = 0x00;

    // Fault Bitmap = 0x00000003 -> bytes 14-17
    frame_bytes[17] = 0x03;

    // Sequence = 1200 (0x04B0) -> bytes 18-19
    frame_bytes[18] = 0x04;
    frame_bytes[19] = 0xB0;

    // Uptime = 3600s (0x00000E10) -> bytes 20-23
    frame_bytes[22] = 0x0E;
    frame_bytes[23] = 0x10;

    TelemetryData decoded;
    bool status = decode_telemetry(frame_bytes, sizeof(frame_bytes), decoded);

    ASSERT_TRUE(status);
    EXPECT_DOUBLE_EQ(decoded.rpm, 3000.0);
    EXPECT_DOUBLE_EQ(decoded.coolant_temp, 85.5);
    EXPECT_DOUBLE_EQ(decoded.oil_pressure, 450.0);
    EXPECT_DOUBLE_EQ(decoded.battery_voltage, 13.84);
    EXPECT_DOUBLE_EQ(decoded.fuel_level, 75.2);
    EXPECT_DOUBLE_EQ(decoded.vehicle_speed, 100.5);
    EXPECT_DOUBLE_EQ(decoded.ambient_temp, 22.0);
    EXPECT_EQ(decoded.fault_bitmap, 3u);
    EXPECT_EQ(decoded.sequence_counter, 1200);
    EXPECT_EQ(decoded.uptime, 3600u);
}

TEST(GatewayCodecTests, TestEncodeCommand) {
    RawCommandFrame frame;
    encode_command(0x02, 75, frame);

    EXPECT_EQ(frame.command_id, 0x02);
    EXPECT_EQ(frame.param_u8, 75);
    for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(frame.reserved[i], 0);
    }
}

TEST(GatewayCodecTests, TestDecodeResponse) {
    RawResponseFrame frame;
    frame.command_id = 0x04;
    frame.status = 0x02;
    memset(frame.reserved, 0, sizeof(frame.reserved));

    uint8_t cmd = 0;
    uint8_t stat = 0;
    bool status = decode_response(reinterpret_cast<uint8_t*>(&frame), sizeof(frame), cmd, stat);

    ASSERT_TRUE(status);
    EXPECT_EQ(cmd, 0x04);
    EXPECT_EQ(stat, 0x02);
}
