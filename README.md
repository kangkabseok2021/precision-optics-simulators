# Precision Optics Simulators

A C++20 monorepo housing high-precision optical simulation utilities.

## Suite Components

1. **[High-Precision Optical Positioning Simulator](high_precision_optical_simulator/)**: 
   - A C++20 numerical engine simulating manufacturing toolpath generation (thomas-algorithm spline solver), actuator/spindle hardware abstractions, dynamic telemetry logging, and state-machine lifecycle control.
2. **[Monte Carlo Optical Ray Tracer for Automotive Lighting](mc_optical_ray_tracer/)** *(Planned)*:
   - A C++20 path tracer featuring Möller-Trumbore ray-triangle intersections, BVH/AABB spatial acceleration, Fresnel optics, cosine hemisphere sampling, and tile-based multi-threaded execution.

---

## Directory Structure

```
precision-optics-simulators/
├── CMakeLists.txt                       # Monorepo build orchestrator
├── README.md                            # Project documentation
├── .gitignore                           # Git ignores (builds, telemetry outputs)
│
├── high_precision_optical_simulator/    # Positioning simulator subproject
│   ├── CMakeLists.txt
│   ├── include/                         # Header files (hal/ and optical/)
│   ├── src/                             # Source implementation files
│   └── tests/                           # GoogleTest suite files
│
└── mc_optical_ray_tracer/               # Ray tracer subproject (planned)
```

---

## Build and Verification

### Prerequisites
- CMake 3.25+
- A C++20 compiler (GCC 11+, Clang 13+, MSVC 2022)
- Git (for fetching GoogleTest)

### Compiling
Configure and compile all targets using the top-level CMake configuration:

```bash
# Configure the build system
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile all libraries and test executables
cmake --build build -j$(nproc)
```

### Running Tests
Execute the GoogleTest suite:

```bash
cd build
ctest --output-on-failure
```

---

## Continuous Integration
A GitHub Actions workflow is set up under `.github/workflows/ci.yml` to compile and run tests on every commit/PR targeting Ubuntu.
