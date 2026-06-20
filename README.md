# LineFollower

STM32F407VGT6 循跡車韌體，使用 C11、STM32 HAL/CMSIS、CMake、Ninja 和
Arm GNU Toolchain 建置。

## 環境需求

- CMake 3.22+
- Ninja
- Arm GNU Toolchain (`arm-none-eabi-gcc`, `gdb`, `objcopy`, `size`)
- OpenOCD（燒錄與偵錯）
- VS Code 建議擴充：CMake Tools、C/C++、Cortex-Debug

以上命令須能直接從終端機執行（位於 `PATH`）。

## 建置

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 建置：

```powershell
cmake --preset Release
cmake --build --preset Release
```

輸出位於 `build/Debug` 或 `build/Release`：

- `LineFollower.elf`：偵錯及 OpenOCD 燒錄
- `LineFollower.hex`：Intel HEX
- `LineFollower.bin`：原始韌體映像
- `LineFollower.map`：連結映射
- `compile_commands.json`：程式碼索引

## 燒錄

連接 CMSIS-DAP/SWD 燒錄器後執行：

```powershell
.\flash.bat
```

腳本會先配置（若需要）、編譯，再用 OpenOCD 燒錄並校驗。

## VS Code

開啟資料夾後選擇 `Debug` CMake preset。`Ctrl+Shift+B` 可編譯，
執行 `Debug (OpenOCD + GDB)` 可編譯、燒錄並在 `main` 停下。

若工具鏈不在 `PATH`，可明確指定安裝根目錄：

```powershell
cmake --preset Debug -DARM_TOOLCHAIN_ROOT="C:/path/to/Arm GNU Toolchain"
```

## 韌體結構

- `Core/Src/main.c`：主流程、GPIO、循跡狀態及 OLED 顯示
- `Core/Src/sensors.c`：8 路灰階感測器與線位置計算
- `Core/Src/motor.c`：雙馬達 PWM/方向及 ESC 控制
- `Core/Src/encoder.c`：雙編碼器速度、計數器回繞與里程計算
- `Core/Src/speed_control.c`：左右輪前饋加 PI 速度閉環
- `Core/Src/pid.c`：通用 PID 控制器
- `Core/Inc/pinout.h`：硬體腳位定義
- `Drivers/`：STM32F4 HAL 與 CMSIS
