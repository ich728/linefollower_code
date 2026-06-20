#!/bin/bash
set -e

cd "$(dirname "$0")"

command -v cmake >/dev/null || { echo "ERROR: cmake was not found on PATH."; exit 1; }
command -v openocd >/dev/null || { echo "ERROR: openocd was not found on PATH."; exit 1; }

if [ ! -f build/Debug/build.ninja ]; then
    echo "=== Configure ==="
    cmake --preset Debug
fi

echo "=== Build ==="
cmake --build --preset Debug

echo ""
echo "=== Flash ==="
openocd \
    -f interface/cmsis-dap.cfg \
    -c "transport select swd" \
    -c "adapter speed 100" \
    -f target/stm32f4x.cfg \
    -c "init" \
    -c "reset halt" \
    -c "program build/Debug/LineFollower.elf verify reset exit"

echo ""
echo "=== DONE ==="
