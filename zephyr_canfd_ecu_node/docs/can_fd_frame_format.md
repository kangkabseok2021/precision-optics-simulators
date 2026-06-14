# CAN-FD Frame Format Specification

This document defines the byte-level contract shared between the Zephyr RTOS ECU telemetry node and the Linux C++20 raw SocketCAN gateway.

All multi-byte integers are packed in **big-endian** (network byte order).

---

## 1. Telemetry Frame (ID: `0x100`)
- **Type**: CAN-FD Frame
- **DLC**: `14` (Payload size: 48 bytes)
- **Settings**: BRS (Bit Rate Switch) enabled

### Byte Layout
| Offset | Type | Name | Scale / Unit | Description |
|---|---|---|---|---|
| `0-1` | `uint16_t` | `rpm` | `1 RPM / LSB` | Engine speed (0 - 8000 RPM) |
| `2-3` | `int16_t` | `coolant_temp` | `Q8.8 / °C` | Coolant temperature (-40 to 150 °C) |
| `4-5` | `uint16_t` | `oil_pressure` | `1 kPa / LSB` | Engine oil pressure (0 - 1000 kPa) |
| `6-7` | `uint16_t` | `battery_voltage` | `0.01 V / LSB` | Battery voltage (e.g. `1260` = 12.60 V) |
| `8-9` | `uint16_t` | `fuel_level` | `0.1 % / LSB` | Fuel tank level (e.g. `985` = 98.5%) |
| `10-11` | `uint16_t` | `vehicle_speed` | `0.1 km/h / LSB` | Vehicle speed (e.g. `1200` = 120.0 km/h) |
| `12-13` | `int16_t` | `ambient_temp` | `Q8.8 / °C` | Ambient environment temperature |
| `14-17` | `uint32_t` | `fault_bitmap` | `Bitmap` | Active system faults (see Fault Bitmap section) |
| `18-19` | `uint16_t` | `sequence_counter` | `1 / LSB` | Monotonically increasing counter, wraps at 65535 |
| `20-23` | `uint32_t` | `uptime` | `1 s / LSB` | Controller uptime in seconds |
| `24-47` | `uint8_t[24]` | `reserved` | N/A | Zero-padding to align with 48-byte DLC 14 size |

### Fault Bitmap Fields
- **Bit 0** (`0x00000001`): `FAULT_OVER_TEMP` (Coolant temperature > 110 °C)
- **Bit 1** (`0x00000002`): `FAULT_LOW_OIL_PRESSURE` (Oil pressure < 150 kPa when RPM > 800)
- **Bit 2** (`0x00000004`): `FAULT_LOW_BATTERY` (Battery voltage < 11.0 V)
- **Bit 3** (`0x00000008`): `FAULT_BUS_OFF` (CAN controller encountered Bus-Off state)

---

## 2. Command Frame (ID: `0x200`)
- **Type**: Classic CAN (or CAN-FD compatible)
- **DLC**: `8` (Payload size: 8 bytes)

### Byte Layout
| Offset | Type | Name | Description |
|---|---|---|---|
| `0` | `uint8_t` | `command_id` | Operation identifier (see Command List below) |
| `1` | `uint8_t` | `param_u8` | Optional parameter byte dependent on command |
| `2-7` | `uint8_t[6]` | `reserved` | Padding, set to `0x00` |

### Command List
* **`0x01` — `SET_MODE`**
  - Sets ECU state. `param_u8` values:
    - `0`: INIT
    - `1`: NORMAL
    - `2`: DIAGNOSTIC
    - `3`: FAULT
* **`0x02` — `SET_THROTTLE`**
  - Sets physics throttle input. `param_u8` range: `0 - 100` (representing %).
* **`0x03` — `REQUEST_DIAG`**
  - Manually transitions the ECU FSM from `NORMAL` into `DIAGNOSTIC` state.
* **`0x04` — `CLEAR_FAULTS`**
  - Commands the state machine to attempt clearing active faults and return to `NORMAL`.

---

## 3. Command Response Frame (ID: `0x201`)
- **Type**: Classic CAN
- **DLC**: `8` (Payload size: 8 bytes)

### Byte Layout
| Offset | Type | Name | Description |
|---|---|---|---|
| `0` | `uint8_t` | `command_id` | Echoes the command being acknowledged |
| `1` | `uint8_t` | `status` | `0` = SUCCESS, `1` = INVALID_PARAM, `2` = FORBIDDEN_STATE |
| `2-7` | `uint8_t[6]` | `reserved` | Padding, set to `0x00` |
