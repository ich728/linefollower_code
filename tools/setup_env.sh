#!/bin/bash
# ═══════════════════════════════════════════
#  學界循跡 — Bash 開發環境設定
#  用法: source tools/setup_env.sh
# ═══════════════════════════════════════════

echo "═╣ 搜尋 ARM GCC 工具鏈..."

ARM_PATH=""

# 1. ARM GNU Toolchain 獨立安裝
for d in "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/"*/; do
    if [ -f "${d}bin/arm-none-eabi-gcc.exe" ]; then
        ARM_PATH="${d}bin"
    fi
done

# 2. STM32CubeIDE 內建 (找最新版)
if [ -z "$ARM_PATH" ]; then
    for d in "C:/ST/STM32CubeIDE_"*; do
        TOOLS=$(find "$d" -path "*/gnu-tools-for-stm32*/tools/bin/arm-none-eabi-gcc.exe" 2>/dev/null | head -1)
        if [ -n "$TOOLS" ]; then
            ARM_PATH=$(dirname "$TOOLS")
        fi
    done
fi

# 3. 在 PATH 中找
if [ -z "$ARM_PATH" ]; then
    FOUND=$(which arm-none-eabi-gcc 2>/dev/null)
    if [ -n "$FOUND" ]; then
        ARM_PATH=$(dirname "$FOUND")
    fi
fi

if [ -z "$ARM_PATH" ]; then
    echo "❌ 找不到 arm-none-eabi-gcc"
    echo "   下載: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads"
else
    echo "✅ ARM GCC: $ARM_PATH"
    export PATH="$ARM_PATH:$PATH"
fi

# ═══════════════════════════════════════════
# OpenOCD
# ═══════════════════════════════════════════
OPENOCD=$(which openocd 2>/dev/null)
if [ -z "$OPENOCD" ]; then
    # CubeIDE 內建
    for d in "C:/ST/STM32CubeIDE_"*; do
        FOUND=$(find "$d" -name "openocd.exe" -path "*/bin/*" 2>/dev/null | head -1)
        if [ -n "$FOUND" ]; then
            export PATH="$PATH:$(dirname "$FOUND")"
            echo "✅ OpenOCD: $(dirname "$FOUND")"
            break
        fi
    done
else
    echo "✅ OpenOCD: $(dirname "$OPENOCD")"
fi

# ═══════════════════════════════════════════
# Verify
# ═══════════════════════════════════════════
echo ""
echo "── 工具鏈驗證 ──"
arm-none-eabi-gcc --version 2>/dev/null | head -1 || echo "❌ gcc"
cmake --version 2>/dev/null | head -1 || echo "❌ cmake (請安裝或從 STM32CubeIDE 終端機執行)"
openocd --version 2>/dev/null | head -1 || echo "⚠️  openocd (燒錄需此工具)"
echo "── 完成 ──"
