# Precision Optics Simulators

A C++20 monorepo housing high-precision optical simulation utilities.

## Suite Components

1. **[High-Precision Optical Positioning Simulator](high_precision_optical_simulator/)**: 
   - A C++20 numerical engine simulating manufacturing toolpath generation (thomas-algorithm spline solver), actuator/spindle hardware abstractions, dynamic telemetry logging, and state-machine lifecycle control.
2. **[Monte Carlo Optical Ray Tracer for Automotive Lighting](mc_optical_ray_tracer/)**:
   - A C++20 path tracer featuring Möller-Trumbore ray-triangle intersections, BVH/AABB spatial acceleration, Fresnel optics, cosine hemisphere sampling, and tile-based multi-threaded execution.

---

## Directory Structure

```
precision-optics-simulators/
├── CMakeLists.txt                       # Monorepo build orchestrator
├── README.md                            # Project documentation
├── .gitignore                           # Git ignores (builds, telemetry outputs)
├── docs/                                # Centralized documentation directory
│   └── OPTICS-MATH.md                   # Math & physics formulas and derivations
│
├── high_precision_optical_simulator/    # Positioning simulator subproject
│   ├── CMakeLists.txt
│   ├── include/                         # Header files (hal/ and optical/)
│   ├── src/                             # Source implementation files
│   └── tests/                           # GoogleTest suite files
│
└── mc_optical_ray_tracer/               # Ray tracer subproject
    ├── CMakeLists.txt
    ├── include/                         # Header files (geometry, optics, tracer, lighting)
    ├── src/                             # Source implementation files
    ├── scripts/                         # Python analysis scripts
    └── tests/                           # GoogleTest suite files
```

---

## Build and Verification

### Prerequisites
- CMake 3.25+
- A C++20 compiler (GCC 11+, Clang 13+, MSVC 2022)
- Git (for fetching GoogleTest)
- Python 3 with `numpy` and `matplotlib` (for ray-tracer luminance analysis)

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

## Running the Simulators

### High-Precision Optical Positioning Simulator
This subproject is organized as an engine core library (`optical_core`). Its components, hardware abstractions, and state-machine logic are verified via its extensive GoogleTest suite. Run:
```bash
./build/high_precision_optical_simulator/test_optical
```

### Monte Carlo Optical Ray Tracer
You can run the ray tracer executable to render a predefined Cornell-box scene:

```bash
# Run the ray tracer
./build/mc_optical_ray_tracer/ray_tracer

# This generates 'render.ppm' in your current working directory.
```

To analyze the rendering's luminance map and generate a distribution histogram:

```bash
# Install required Python packages if not already present
pip install numpy matplotlib

# Analyze the generated image
python3 mc_optical_ray_tracer/scripts/analyze_luminance.py render.ppm
```

This prints min, max, and mean luminance metrics to the console and outputs a comparison visualization at `render_analysis.png`.

---

## Continuous Integration
A GitHub Actions workflow is set up under `.github/workflows/ci.yml` to compile and run tests on every commit/PR targeting Ubuntu.

