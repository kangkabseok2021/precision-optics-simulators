# IMU Breakout — STM32H743 + ICM-42688-P

Companion documentation for `hardware/imu_breakout.kicad_sch`. This is the
sensor front-end this project's `InferenceEngine`/`AttitudePid` code is
written against: a Cortex-M7 flight controller reading a 6-axis IMU over
SPI at high rate for attitude estimation, plus the 16-beam LiDAR array
that feeds the obstacle-detection MLP (LiDAR module wiring is out of
scope for this schematic — this sheet covers only the IMU breakout).

## Parts

| Ref  | Part            | Package         | Notes                              |
|------|-----------------|-----------------|-------------------------------------|
| U1   | STM32H743VITx   | LQFP-100        | Cortex-M7, 480 MHz, hard-float FPU |
| U2   | ICM-42688-P     | LGA-14 (2.5x3mm)| 6-axis IMU, SPI mode 3, up to 32 kHz|
| C1   | 100 nF          | 0402            | U1 VDD decoupling                  |
| C2   | 100 nF          | 0402            | U1 VDDIO decoupling                |
| C3   | 10 µF           | 0805            | U1 bulk decoupling (VDD rail)      |
| C4   | 100 nF          | 0402            | U2 VDD decoupling                  |
| J1   | 10-pin 0.05" IDC| SWD header      | ST-Link programming/debug          |

## SPI2 wiring (U1 STM32H743 ↔ U2 ICM-42688-P)

| Signal      | STM32H743 pin | ICM-42688-P pin | Notes                          |
|-------------|---------------|-----------------|--------------------------------|
| SPI2_SCK    | PB13          | SCLK            | Max 24 MHz (register), 8 MHz (burst)|
| SPI2_MISO   | PB14          | AD0/SDO         | IMU → MCU                      |
| SPI2_MOSI   | PB15          | SDI             | MCU → IMU                      |
| SPI2_NSS    | PB12 (GPIO CS)| CS              | Software-controlled chip select|
| INT1        | PB0 (EXTI0)   | INT1            | Data-ready interrupt, active high|

Rationale for software-CS over hardware NSS: the IMU read sequence needs
a single, uninterrupted chip-select assertion across the register-address
byte and all data bytes, which is simpler to guarantee with GPIO bit-bang
CS than with the SPI peripheral's automatic NSS management.

## Decoupling

- 100 nF placed within 0.5 mm of every VDD/VDDIO pin on both U1 and U2 —
  standard high-frequency decoupling for MCU and MEMS-IMU supply rails.
- 10 µF bulk capacitor (C3) near the STM32H743's main 3.3V input, sized
  for the current transients of the M7 core at 480 MHz.

## SPI trace length matching

SCK/MISO/MOSI are routed as a matched group (±2 mm) to keep clock-to-data
skew well under one bit period at the IMU's 24 MHz register-access clock;
this is a soft-real-time constraint, not a hard timing closure requirement
at these speeds; it exists to avoid data corruption on marginal boards.

## Noise density comparison

ICM-42688-P datasheet gyro noise density is ~2.8 mdps/√Hz — roughly 3x
lower than the ICM-20602 predecessor used in earlier iterations of this
concept, which is the reason for the part choice: lower noise floor
directly reduces the attitude-estimate jitter the `AttitudePid` loop has
to reject.

## IMU centre placement

U2 is placed as close to the airframe's centre of gravity/rotation as
the layout allows, to minimize the lever-arm between the IMU's sense
axes and the vehicle's true rotation centre — off-centre placement adds
a centripetal-acceleration bias term to the accelerometer reading during
yaw that this project's PID loop does not compensate for.

## SWD header (J1)

Standard 10-pin 0.05" ARM Cortex debug connector (SWDIO/SWCLK/GND/3V3/
NRST/SWO), wired directly to the STM32H743's SWD pins (PA13/PA14) for
ST-Link programming and live debugging during bring-up.
