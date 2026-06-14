# Zephyr RTOS CAN-FD ECU Telemetry Node & C++20 Linux Gateway

This subproject implements a dual-sided automotive simulation stack:
1. **ECU Simulator**: A Zephyr RTOS application running on the `native_sim` architecture. It models engine dynamics, maintains an internal state machine (INIT, NORMAL, DIAGNOSTIC, FAULT), and broadcasts high-speed CAN-FD telemetry frames.
2. **Telemetry Gateway**: A Linux service written in modern C++20. It binds to the CAN-FD socket, decodes telemetry frames, publishes updates via MQTT, and hosts an HTTPS REST server exposing the latest data.

---

## 1. Directory Structure

```
zephyr_canfd_ecu_node/
├── README.md                      # This documentation
├── Dockerfile.zephyr              # Container compile target for Zephyr RTOS app
├── Dockerfile.gateway             # Container deployment target for C++20 Gateway
├── docker-compose.yml             # Orchestration for gateway + MQTT broker
│
├── docs/
│   ├── can_fd_frame_format.md     # Bitwise format layout spec
│   └── uml/                       # PlantUML source files for FSM & component diagrams
│
├── ecu/                           # Zephyr RTOS ECU firmware
│   ├── CMakeLists.txt
│   ├── prj.conf                   # Kernel and device feature configurations
│   ├── boards/
│   │   └── native_sim.overlay     # Devicetree overlay linking CAN nodes to vcan0
│   ├── src/                       # FSM, Physics engine, and CAN codecs
│   └── tests/                     # twister unit tests running on native_sim
│
└── gateway/                       # Linux C++20 Gateway
    ├── CMakeLists.txt
    ├── include/                   # SocketCAN, MQTT, and HTTPS modules
    ├── src/                       # Main bootstrap and module implementations
    └── tests/                     # GoogleTest cases
```

---

## 2. Path to Real Hardware

While this project is configured to run on `native_sim` using Linux virtual CAN interfaces (`vcan0`), it is designed for direct portability to physical hardware (e.g. an **STM32G474** / **STM32H7** microcontroller with an on-board FDCAN controller).

### 2.1 Devicetree Overlay Customization
To run on a real target, replace the `boards/native_sim.overlay` with your micro's overlay (e.g. `boards/stm32g474e_eval.overlay`):

```devicetree
&fdcan1 {
    status = "okay";
    pinctrl-0 = <&fdcan1_rx_pa11 &fdcan1_tx_pa12>;
    pinctrl-names = "default";
    bus-speed = <500000>;
    bus-speed-data = <2000000>;
    
    /* Enabled hardware bit-rate switching and FD mode */
    sample-point = <800>;
    sample-point-data = <750>;
};
```

### 2.2 On-Chip Debugging & Flashing
Use an SWD/JTAG debugger (such as a Segger J-Link or ST-LINK) with OpenOCD or J-Link GDB Server.
1. Build the binary using the target board:
   ```bash
   west build -b stm32g474e_eval ecu/
   ```
2. Flash the board using `west flash`:
   ```bash
   west flash --runner openocd
   ```
3. Connect an interactive debugger:
   ```bash
   west debug
   ```

### 2.3 Oscilloscope & Bit-Timing Verification
When bringing up CAN-FD on real hardware, it is critical to verify the physical bus signals using a digital storage oscilloscope (DSO) or a CAN logic analyzer:
1. Connect scope channels to the **CAN_H** and **CAN_L** pins on the transceiver output (observing the differential $V_{diff} = V_{CANH} - V_{CANL}$).
2. Trigger on the start-of-frame (SOF) falling edge.
3. Verify that the nominal phase transitions cleanly from recessive ($0\text{ V}$ differential) to dominant ($2\text{ V}$ differential) at $500\text{ kbit/s}$ (bit width $2\text{ }\mu\text{s}$).
4. Verify the Bit Rate Switch (BRS) transition, checking that data phase bits compress to $500\text{ ns}$ width ($2\text{ Mbit/s}$).
5. Check for signal ringing or reflections; ensure the bus is terminated with exactly $120\text{ }\Omega$ resistors at both physical ends of the line.

---

## 3. Build & Running Instructions

### 3.1 Setup virtual CAN bus (`vcan0`) on Linux
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

### 3.2 Compilation & Execution (Docker Compose)
You can run the gateway and the mosquitto broker inside docker containers:
```bash
docker compose up --build -d
```
Then build and run the Zephyr ECU node:
```bash
west build -b native_sim ecu/
./build/zephyr/zephyr.exe
```

---

## 4. Verification and Tests

### 4.1 Twister Unit Tests
```bash
west twister -p unit_testing -T ecu/tests
```

### 4.2 Gateway GoogleTests
```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build --target test_gateway
./build/zephyr_canfd_ecu_node/gateway/test_gateway
```

### 4.3 Python Integration and E2E Pytests
```bash
# Install testing dependencies
pip install pytest python-can paho-mqtt requests

# Run integration tests against vcan0
pytest tests/test_integration.py

# Run end-to-end tests against composing containers
pytest tests/test_e2e.py
```
