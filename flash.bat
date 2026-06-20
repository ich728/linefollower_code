@echo off
setlocal

cd /d %~dp0

where cmake >nul 2>&1 || (
    echo ERROR: cmake was not found on PATH.
    exit /b 1
)
where openocd >nul 2>&1 || (
    echo ERROR: openocd was not found on PATH.
    exit /b 1
)

if not exist build\Debug\build.ninja (
    echo === Configure ===
    cmake --preset Debug
    if errorlevel 1 exit /b %ERRORLEVEL%
)

echo === Build ===
cmake --build --preset Debug
if errorlevel 1 exit /b %ERRORLEVEL%

copy /y build\Debug\compile_commands.json . >nul 2>&1

echo.
echo === Flash ===
openocd -f interface/cmsis-dap.cfg -c "transport select swd" -c "adapter speed 100" -f target/stm32f4x.cfg -c "init" -c "reset halt" -c "program build/Debug/LineFollower.elf verify reset exit"

if %ERRORLEVEL% equ 0 (
    echo.
    echo === DONE ===
) else (
    echo.
    echo === FLASH FAILED - replug CMSIS-DAP and retry ===
)
