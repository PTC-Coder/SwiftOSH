@echo off
REM Clone FreeRTOS kernel and STM32 USB Device Library into Middlewares.
REM Run from the SwiftOSH directory. Requires git on PATH.

REM ---- FreeRTOS ----
if not exist "Middlewares\Third_Party\FreeRTOS\Source" (
    echo Cloning FreeRTOS-Kernel...
    git clone --depth 1 --branch V10.3.1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git Middlewares\Third_Party\FreeRTOS\Source
    if errorlevel 1 (
        echo ERROR: Failed to clone FreeRTOS-Kernel
        goto :freertos_done
    )
    REM Add CMSIS-RTOS v1 wrapper (not included in kernel repo)
    if not exist "Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS" (
        mkdir "Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS"
    )
    echo FreeRTOS cloned. You need to add cmsis_os.c and cmsis_os.h to CMSIS_RTOS folder.
    echo Get them from an STM32CubeL4 or STM32CubeF4 package, or from:
    echo https://github.com/ARM-software/CMSIS-FreeRTOS/tree/master/CMSIS/RTOS
) else (
    echo Already exists: Middlewares\Third_Party\FreeRTOS\Source
)
:freertos_done

REM ---- USB Device Library ----
if not exist "Middlewares\ST\STM32_USB_Device_Library" (
    echo Cloning STM32 USB Device Library...
    git clone --depth 1 --branch v2.11.3 https://github.com/STMicroelectronics/stm32_mw_usb_device.git Middlewares\ST\STM32_USB_Device_Library
    if errorlevel 1 (
        echo ERROR: Failed to clone STM32_USB_Device_Library
        goto :usb_done
    )
    echo USB Device Library cloned.
) else (
    echo Already exists: Middlewares\ST\STM32_USB_Device_Library
)
:usb_done

echo.
echo Done. Check output above for any errors.
pause
