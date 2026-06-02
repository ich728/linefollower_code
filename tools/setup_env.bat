@echo off
chcp 65001 >nul
echo ╔══════════════════════════════════════════╗
echo ║   學界循跡 — 開發環境設定              ║
echo ╚══════════════════════════════════════════╝
echo.

REM ═══════════════════════════════════════════
REM 自動偵測已安裝的 ARM GCC
REM ═══════════════════════════════════════════
set "ARM_PATH="

REM 1. ARM GNU Toolchain (獨立安裝)
for /d %%d in ("C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\*") do (
    if exist "%%d\bin\arm-none-eabi-gcc.exe" set "ARM_PATH=%%d\bin"
)

REM 2. STM32CubeIDE 內建 (常見版本)
if "%ARM_PATH%"=="" (
    for /d %%d in ("C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*\tools\bin") do (
        set "ARM_PATH=%%d"
    )
)

REM 3. 手動指定 (修改這裡)
REM set "ARM_PATH=C:\your\custom\arm-gcc\bin"

if "%ARM_PATH%"=="" (
    echo [錯誤] 找不到 arm-none-eabi-gcc.exe
    echo.
    echo 請下載安裝 ARM GNU Toolchain:
    echo   https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
    echo   選 "arm-none-eabi" → Windows mingw32 版本
    echo.
    pause
    goto :eof
)

echo [OK] ARM GCC: %ARM_PATH%
echo.

REM ═══════════════════════════════════════════
REM 尋找 CMake
REM ═══════════════════════════════════════════
set "CMAKE_FOUND="

REM 1. PATH 中已有
where cmake >nul 2>&1 && set "CMAKE_FOUND=1"

REM 2. STM32CubeIDE 內建
if "%CMAKE_FOUND%"=="" (
    for /d %%d in ("C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.*\bin") do (
        set "PATH=%PATH%;%%d"
        set "CMAKE_FOUND=1"
        echo [OK] CMake: %%d
    )
)

if "%CMAKE_FOUND%"=="" (
    echo [WARN] 找不到 cmake，請安裝:
    echo   https://cmake.org/download/
    echo   或安裝 STM32CubeIDE
)

REM ═══════════════════════════════════════════
REM 尋找 OpenOCD
REM ═══════════════════════════════════════════
set "OPENOCD_FOUND="

where openocd >nul 2>&1 && set "OPENOCD_FOUND=1"

if "%OPENOCD_FOUND%"=="" (
    for /d %%d in ("C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.debug.openocd_*\resources\openocd\bin") do (
        set "PATH=%PATH%;%%d"
        set "OPENOCD_FOUND=1"
        echo [OK] OpenOCD: %%d
    )
    REM 也檢查 CubeProgrammer 的 ST-LINK gdbserver
    for /d %%d in ("C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin") do (
        set "PATH=%PATH%;%%d"
        echo [OK] ST-LINK_gdbserver: %%d
    )
) else (
    echo [OK] OpenOCD already in PATH
)

REM ═══════════════════════════════════════════
REM 設定環境變數
REM ═══════════════════════════════════════════
set "PATH=%ARM_PATH%;%PATH%"

echo.
echo ═══════════════════════════════════════════
echo   環境已就緒，請執行:
echo.
echo     arm-none-eabi-gcc --version
echo.
echo   如果顯示版本資訊，表示設定成功。
echo ═══════════════════════════════════════════
echo.

REM 啟動新的 cmd 保留環境
cmd /k "cd /d %~dp0.. && echo 工作目錄: %CD% && echo. && arm-none-eabi-gcc --version && echo. && echo 可輸入 cmake 指令開始構建"
