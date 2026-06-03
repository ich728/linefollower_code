#!/bin/bash
set -e

CMAKE="C:/Users/ICH728/AppData/Local/stm32cube/bundles/cmake/4.3.1+st.1/bin/cmake.exe"
OPENOCD="C:/Users/ICH728/AppData/Local/openocd/bin/openocd.exe"
SCRIPTS="C:/Users/ICH728/AppData/Local/openocd/share/openocd/scripts"

cd "$(dirname "$0")"

echo "=== Build ==="
"$CMAKE" --build build/Debug

echo ""
echo "=== Flash ==="
"$OPENOCD" -s "$SCRIPTS" \
    -f interface/cmsis-dap.cfg \
    -c "transport select swd" \
    -c "adapter speed 100" \
    -f target/stm32f4x.cfg \
    -c "init" \
    -c "reset halt" \
    -c "program build/Debug/LineFollower.elf verify reset exit"

echo ""
echo "=== DONE ==="
