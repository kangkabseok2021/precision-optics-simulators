# ros2_amr_stack — Differential-Drive Autonomous Mobile Robot Stack

This subproject implements an end-to-end simulated **Differential-Drive Autonomous Mobile Robot (AMR)**. It contains core algorithmic modules written in framework-agnostic C++17, wrapped in ROS2 component node wrappers with zero-copy intra-process message passing.

---

## Subproject Structure

* **`libs/`**: Framework-free C++17 libraries. Fully mockable and decoupled from ROS2 for deterministic testing:
  - `can_protocol`: Velocity-command and encoder-ticks packing and SocketCAN transport.
  - `world_sim_model`: 2D differential-drive dynamics, noise injector, and lidar/camera frame simulator.
  - `perception_pipeline`: PCL obstacle plane removal & OpenCV solvePnP visual tracking.
  - `pose_fusion_ekf`: Extended Kalman Filter fusing odometry updates with marker position fixes.
  - `goal_controller`: Navigation setpoint tracking with a dynamic collision-safety stop.
  - `cloud_telemetry`: Robot-to-cloud telemetry and cloud-to-robot JSON message serializers.
* **`src/`**: ROS2 packages buildable via standard `colcon`:
  - `amr_nodes`: Component nodes (`rclcpp_components`) mapping the libraries above to ROS2 publishers, subscribers, and executors.
  - `amr_bringup`: launch compositions, parameter YAMLs, and `launch_testing` scripts.
* **`scripts/`**: Convenience build tools and Python dashboard visualization CLI (`amr_demo_cli`).
* **`docker/`**: Sandbox configuration running a mosquitto MQTT broker alongside the AMR ROS2 Humble container.

---

## Build & Run Instructions

### 1. Standalone Libraries (CMake & CTest)
Since the `libs/` subfolder utilizes standard CMake, it integrates with the monorepo root. To compile only these targets:

```bash
# Configure the build system from this folder
cmake -S libs -B build_libs -DCMAKE_BUILD_TYPE=Release

# Compile all libraries and test suites
cmake --build build_libs -j$(nproc)

# Run tests
cd build_libs && ctest --output-on-failure
```

### 2. Full Workspace Build (ROS2 & Colcon)
Building the full ROS2 node wrapper requires a ROS2 Humble installation (Ubuntu 22.04 base). Run:

```bash
# Source the ROS2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run unit and integration tests
colcon test --event-handlers console_cohesion+
```

### 3. Running Containerized Simulation (Docker)
A complete sandbox simulation environment (containing ROS2 nodes, MQTT broker, and vcan0 setup) is prepared under the `docker/` directory:

```bash
# Set up host virtual CAN interface
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Launch the docker-compose services
docker compose -f docker/docker-compose.yml up --build
```
This starts the robot simulation and the cloud bridge. You can then run `python3 scripts/amr_demo_cli.py` to stream real-time telemetry and command the robot to destinations.
