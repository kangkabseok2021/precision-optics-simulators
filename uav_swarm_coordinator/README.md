# UAV Swarm Deconfliction Engine

Multi-UAV swarm coordinator: Conflict-Based Search planner, greedy auction task assigner,
and uncertainty-weighted pose consensus filter — implemented in pure Rust.

## Architecture

```
swarm_algorithms/           Pure Rust library crate — no ROS 2 dependency
├── conflict_solver         CBS: optimal collision-free 3D pathfinding for N drones
├── task_assigner           Greedy auction: distributes M goals to K drones
└── pose_consensus          1/σ² weighted-average GPS/VIO position fusion
```

## Quick Start

```bash
# Install Rust (if not installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

cd uav_swarm_coordinator
cargo test          # 39 tests, 0 failures
cargo clippy        # zero warnings enforced
```

## Algorithm Details

### Conflict-Based Search (CBS)

CBS finds globally optimal, collision-free paths for N agents via a two-level search:

- **High level**: constraint tree explored in SIC-cost order (BinaryHeap min-heap)
- **Low level**: time-extended A* per agent, honouring vertex and edge constraints
- **Optimality guarantee**: first conflict-free solution found is globally optimal by SIC

### Greedy Auction Task Assignment

Distributes M delivery goals to K drones iterating goals in index order:

1. For each goal, find the drone with minimum squared Euclidean distance from its current endpoint
2. Assign goal to that drone; update drone's endpoint to that goal
3. Repeat until all goals claimed

**Complexity**: O(K·M). Deterministic for identical inputs.

### Pose Consensus Filter

Uncertainty-weighted average across N drone position broadcasts:

```
weight_k = 1 / max(σ²_k, 1e-9)      (information weight per axis)
fused_k  = Σ(weight_k × pos_k) / Σ(weight_k)
```

The 1/σ² combiner is the MLE-optimal linear estimator for independent Gaussian measurements.

## CI

| Job | What it checks |
|-----|----------------|
| `rust-algorithms` | `cargo test --all` + `cargo clippy -D warnings` |
| `aarch64-cross` | `cargo build --release --target aarch64-unknown-linux-gnu` (Jetson Orin path) |

## Roadmap

- **Phase 2** (planned): `rclrs` (ros2_rust) `SwarmPlannerNode` — subscribe `/mission/goals`,
  publish `/swarm/plan` (DronePathArray) and `/swarm/pose_consensus`
- **Phase 3** (planned): Gazebo Harmonic 3-drone SITL simulation, Docker Compose, mission CLI

