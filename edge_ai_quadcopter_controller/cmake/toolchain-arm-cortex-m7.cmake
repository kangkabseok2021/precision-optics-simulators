# Cross-compilation toolchain for STM32H743 (Cortex-M7, hard-float ABI).
# Usage: cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-cortex-m7.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m7)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR           arm-none-eabi-ar)
set(CMAKE_RANLIB       arm-none-eabi-ranlib)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CPU_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")
# Note: -ffreestanding is intentionally NOT set here. It forces
# __STDC_HOSTED__ (and libstdc++'s _GLIBCXX_HOSTED) to 0, which breaks
# <cmath>'s TR1 special-function headers (bessel/laguerre/zeta) — they
# unconditionally call std::__throw_domain_error, whose declaration is
# gated on hosted mode, so <cmath> fails to compile at all under
# -ffreestanding on this newlib toolchain. We link against newlib's
# full C library anyway (this only builds a static library, not a
# bare-metal boot image), so freestanding mode buys nothing here.
set(CMAKE_C_FLAGS   "${CPU_FLAGS} -Os" CACHE STRING "")
set(CMAKE_CXX_FLAGS "${CPU_FLAGS} -fno-rtti -Os" CACHE STRING "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
