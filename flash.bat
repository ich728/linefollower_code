@echo off
setlocal

set CMAKE=C:\Users\ICH728\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe
set OPENOCD=C:\Users\ICH728\AppData\Local\openocd\bin\openocd.exe
set SCRIPTS=C:\Users\ICH728\AppData\Local\openocd\share\openocd\scripts

cd /d %~dp0

echo === Build ===
%CMAKE% --build build\Debug
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

copy /y build\Debug\compile_commands.json . >nul 2>&1

echo.
echo === Flash ===
%OPENOCD% -s "%SCRIPTS%" -f interface/cmsis-dap.cfg -c "transport select swd" -c "adapter speed 100" -f target/stm32f4x.cfg -c "init" -c "reset halt" -c "program build/Debug/LineFollower.elf verify reset exit"

if %ERRORLEVEL% equ 0 (
    echo.
    echo === DONE ===
) else (
    echo.
    echo === FLASH FAILED - replug CMSIS-DAP and retry ===
)
pause
