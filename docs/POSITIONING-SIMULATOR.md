# High-Precision Optical Positioning Simulator

This component simulates the manufacturing toolpath generation, motion control logic, and telemetry collection for precision lathe or grinding machine systems used in optical lens surfacing.

---

## Core Components

### 1. Thomas-Algorithm Cubic Spline Interpolation
To generate smooth manufacturing toolpaths, target coordinates on a lens blank must be continuous and twice-differentiable.
- **Problem**: Traditional cubic-spline fitting requires solving a system of linear equations to determine spline derivatives. Solving this directly using standard LU decomposition or matrix inversion takes $O(N^3)$ operations.
- **Solution**: The spline boundary conditions result in a tridiagonal matrix system. We implement a natural cubic spline solver utilizing the **Thomas Algorithm**, which solves tridiagonal systems in $O(N)$ linear time.
- **Performance**: pre-computes the spline coefficients during generator initialization, allowing the simulator to interpolate the target sag height ($z$ in micrometers) at any arbitrary radial position ($r$ in millimeters) in $O(1)$ constant time during real-time machining loop execution.

### 2. Lock-Free SPSC Telemetry Logger
Manufacturing control loops operate at high frequencies (e.g., $1\text{ kHz}$) and must run with hard real-time guarantees. Blocking the main thread for I/O operations (such as writing to a disk or database) is unacceptable and causes machining jitter.
- **Architecture**: A lock-free **Single-Producer Single-Consumer (SPSC)** ring buffer.
- **Producer (Control Loop)**: Pushes telemetry state frames (`Frame` structs containing timestamp, 3D axes coordinates, spindle speed, error deviation, and event status codes). The `push()` operation performs atomic index additions and never locks or blocks. If the buffer is temporarily full, it increments a atomic dropped frame counter and returns immediately.
- **Consumer (Writer Thread)**: Runs on a background thread. It polls the ring buffer, drains pending telemetry frames, and writes them to a structured CSV file at a regular interval (every 10 ms).
- **Graceful Shutdown**: The `stop()` call signals the writer thread, flushes any remaining items in the buffer, and joins the background thread to prevent data loss or resource leaks.

### 3. Programmable Logic Controller (PLC) Lifecycle FSM
Mimics industrial PLC state machine lifecycles to control and monitor hardware components.
- **States**: `IDLE` $\rightarrow$ `INITIALISING` $\rightarrow$ `CALIBRATING` $\rightarrow$ `READY` $\rightarrow$ `ACTIVE`. If any hardware failure occurs or an interlock opens, the system transitions immediately to `FAULT`.
- **Transitions**:
  - `start()`: Initializes all devices.
  - `calibrate()`: Performs self-testing on all axes and sensors.
  - `activate()`: Enters active machining loop state.
  - `fault()`: Safely executes immediate shutdown of all connected actuators.
  - `reset()`: Recovers from a fault state back to `IDLE` after checking safety criteria.

### 4. Hardware Abstraction Layer (HAL)
Simulates physical lathe components:
- **LinearAxis**: Simulates high-precision lead screw stages. Simulates linear positioning, velocity tracking, and limits.
- **SpindleMotor**: Tracks spindle rotation speed (RPM), startup acceleration ramp-up, and electrical current draw under machining load.
- **PressureSensor**: Verifies pneumatic clamping pressure and cutting coolant fluid flow.
- **SafetyInterlock**: Monitors the physical enclosure shields, raising an immediate fault trigger if safety shields are breached during spindle activity.
