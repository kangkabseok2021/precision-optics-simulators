# Edge AI Quadcopter Attitude Controller

A constrained embedded C++17 flight-controller simulation combining a
deterministic PID attitude controller with an on-device, TFLM-style INT8
obstacle-detection MLP. Every algorithmic type (`PidController`,
`AttitudePid`, `MotorMixer`, `InferenceEngine`) is header-only and
compiles unmodified for the host (GoogleTest/Google Benchmark) and for
an ARM Cortex-M7 cross target — same source, two toolchains.

## Directory structure

```
edge_ai_quadcopter_controller/
├── CMakeLists.txt              # host build (tests/benchmark) + ARM cross build
├── cmake/
│   └── toolchain-arm-cortex-m7.cmake
├── include/edge_ai_quadcopter/
│   ├── pid_controller.h        # single-axis PID, constexpr-constructible
│   ├── attitude_pid.h          # 3x PidController value members (pitch/roll/yaw)
│   ├── motor_mixer.h           # X-frame throttle/pitch/roll/yaw -> 4 motor outputs
│   ├── inference_engine.h      # static-arena INT8 MLP inference (see below)
│   └── model_data.h            # extern declarations for the generated model
├── src/
│   ├── model_data.cc           # GENERATED — do not hand-edit
│   ├── main_sim.cpp             # host demo entry point
│   ├── arm_smoke.cpp            # ARM cross-compile smoke target
│   └── benchmark_main.cpp       # Google Benchmark harness
├── tests/                       # GoogleTest: PID, mixer, inference engine
├── scripts/
│   ├── generate_model.py        # trains + INT8-quantizes the model, writes model_data.cc
│   └── benchmark.sh             # asserts the combined loop is under 500 us
└── hardware/
    ├── imu_breakout.kicad_sch   # STM32H743 + ICM-42688-P SPI wiring
    └── hardware-design.md       # pinout, decoupling, and layout rationale
```

## On TFLite Micro

The design intent here is a TFLM-style engine: a fixed-size static arena
holding runtime activation tensors, constructed via placement-new so
there is zero heap allocation once `init()` has run, with weights
quantized to INT8 offline and the same quantize -> integer-matmul ->
dequantize round trip TFLM uses on real Cortex-M hardware.

`InferenceEngine` in `include/edge_ai_quadcopter/inference_engine.h` is
**our own hand-written implementation** of that architecture, not the
upstream `tensorflow/tflite-micro` library. That project ships no CMake
build at all (Bazel/Make only), so it can't be pulled in with
`FetchContent` for a build that also has to cross-compile cleanly to
Cortex-M7 in CI. `generate_model.py` trains the 16-8-4-1 MLP with
hand-written NumPy backprop (no TensorFlow/Keras dependency) and performs
the same INT8 post-training quantization math (per-tensor symmetric
weights, per-tensor asymmetric activations) that TFLite's converter does,
writing the result straight into `model_data.cc`.

## Build & test (host)

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure   # 16 GoogleTests
./build/main_sim                              # scripted demo (5 ticks, one obstacle)
```

## Cross-compile (Cortex-M7 / STM32H743)

Requires a full `arm-none-eabi-gcc` toolchain with newlib + libstdc++
(the Ubuntu `gcc-arm-none-eabi` apt package, or the official ARM GNU
Toolchain release — Homebrew's `arm-none-eabi-gcc` formula on macOS ships
the compiler only, without newlib/libstdc++ headers, and will fail to
compile `<cstddef>`/`<cmath>`).

```sh
cmake -S . -B build-arm \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-cortex-m7.cmake \
  -DCMAKE_BUILD_TYPE=MinSizeRel
cmake --build build-arm -j
arm-none-eabi-size build-arm/libeaq_core.a build-arm/libeaq_arm_smoke.a
```

This builds `eaq_arm_smoke` as a **static library**, not a linked
executable — there's no vector table, linker script, or startup code,
because the goal is proving the full algorithmic core (PID, mixer,
inference) compiles under the ARM hard-float ABI and reporting its
`.text`/`.data`/`.bss` footprint, not producing a bootable image.

Note: the toolchain file deliberately does **not** pass `-ffreestanding`.
That flag sets `__STDC_HOSTED__=0`, which flips libstdc++'s
`_GLIBCXX_HOSTED` off and breaks `<cmath>`'s TR1 special-function headers
(they unconditionally call `std::__throw_domain_error`, whose declaration
is gated on hosted mode) — a real incompatibility discovered while
bringing this target up. Since this only builds a static library against
newlib's full C library rather than a bare-metal boot image, freestanding
mode buys nothing here.

## Benchmark (500 us real-time deadline)

```sh
./scripts/benchmark.sh
```

Builds a Release binary with Google Benchmark, runs `PID+Mixer` and
`TFLM Inference` for 11 repetitions each, and fails (`exit 1`) if their
combined median exceeds 500 us — the budget for a 250 Hz attitude-control
loop.

## Regenerating the model

```sh
python3 scripts/generate_model.py
```

Deterministic given the default seed (20K synthetic 16-beam LiDAR scans,
class-balanced with contiguous 3-6-beam obstacle arcs — an obstacle
subtends multiple adjacent beam angles physically, and scattering 1-3
independently-chosen beams among 16 inputs turned out to plateau around
70% accuracy with this network's capacity). Current run: ~95.7% training
accuracy.
