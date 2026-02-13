@echo off
REM Run this script from the SwiftOSH directory to create directory junctions
REM for HAL/CMSIS from the STM32CubeU5 firmware package.
REM
REM FreeRTOS and USB Device Library are NOT in the U5 V1.7.0 package —
REM use setup_middleware.bat to clone those separately.

set CUBE_U5_DIR=C:\Users\ptcha\STM32Cube\Repository\STM32Cube_FW_U5_V1.7.0

REM HAL Driver
if not exist "Drivers\STM32U5xx_HAL_Driver" (
    mklink /J "Drivers\STM32U5xx_HAL_Driver" "%CUBE_U5_DIR%\Drivers\STM32U5xx_HAL_Driver"
    echo Created junction: Drivers\STM32U5xx_HAL_Driver
) else (
    echo Already exists: Drivers\STM32U5xx_HAL_Driver
)

REM CMSIS
if not exist "Drivers\CMSIS" (
    mklink /J "Drivers\CMSIS" "%CUBE_U5_DIR%\Drivers\CMSIS"
    echo Created junction: Drivers\CMSIS
) else (
    echo Already exists: Drivers\CMSIS
)

echo.
echo HAL/CMSIS junctions created.
echo FreeRTOS and USB Device Library must be cloned separately — see setup_middleware.bat
pause
