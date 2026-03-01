##############################################################################
# SwiftOSH Makefile — STM32U545RET6Q, GCC (arm-none-eabi)
#
# Prerequisites:
#   1. arm-none-eabi-gcc toolchain on PATH
#   2. STM32CubeU5 firmware package — set CUBE_U5_DIR below
#   3. FreeRTOS kernel — set FREERTOS_DIR below
#   4. STM32 USB Device Library — set USB_LIB_DIR below
#
# Usage:
#   make                  # build
#   make clean            # clean
#   make flash            # flash via ST-Link (requires STM32_Programmer_CLI)
##############################################################################

TARGET = SwiftOSH

# ---- Toolchain ----
PREFIX = arm-none-eabi-
CC  = $(PREFIX)gcc
AS  = $(PREFIX)gcc -x assembler-with-cpp
CP  = $(PREFIX)objcopy
SZ  = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# ---- Paths — EDIT THESE to match your local STM32CubeU5 installation ----
CUBE_U5_DIR   ?= ../STM32CubeU5
HAL_DIR        = $(CUBE_U5_DIR)/Drivers/STM32U5xx_HAL_Driver
CMSIS_DIR      = $(CUBE_U5_DIR)/Drivers/CMSIS
CMSIS_DEV_DIR  = $(CMSIS_DIR)/Device/ST/STM32U5xx
FREERTOS_DIR  ?= $(CUBE_U5_DIR)/Middlewares/Third_Party/FreeRTOS/Source
USB_LIB_DIR   ?= $(CUBE_U5_DIR)/Middlewares/ST/STM32_USB_Device_Library

# ---- Build directory ----
BUILD_DIR = build

# ---- MCU flags ----
CPU   = -mcpu=cortex-m33
FPU   = -mfpu=fpv5-sp-d16
FLOAT = -mfloat-abi=hard
MCU   = $(CPU) -mthumb $(FPU) $(FLOAT)

# ---- Defines ----
DEFS = \
  -DSTM32U545xx \
  -DUSE_HAL_DRIVER \
  -DUSE_FULL_LL_DRIVER

# ---- Include paths ----
INCLUDES = \
  -ICore/Inc \
  -IDrivers/SwiftOSH_Drivers/Inc \
  -IMiddlewares/Third_Party/FatFs/src \
  -I$(HAL_DIR)/Inc \
  -I$(HAL_DIR)/Inc/Legacy \
  -I$(CMSIS_DIR)/Include \
  -I$(CMSIS_DEV_DIR)/Include \
  -I$(FREERTOS_DIR)/include \
  -I$(FREERTOS_DIR)/CMSIS_RTOS \
  -I$(FREERTOS_DIR)/portable/GCC/ARM_CM33_NTZ/non_secure \
  -I$(USB_LIB_DIR)/Core/Inc \
  -I$(USB_LIB_DIR)/Class/CustomHID/Inc

# ---- C sources ----
C_SOURCES = \
  Core/Src/main.c \
  Core/Src/stm32u5xx_it.c \
  Core/Src/stm32u5xx_hal_msp.c \
  Core/Src/freertos.c \
  Core/Src/system_stm32u5xx.c \
  Core/Src/usb_device.c \
  Core/Src/usbd_conf.c \
  Core/Src/usbd_desc.c \
  Core/Src/usbd_custom_hid_if.c \
  Drivers/SwiftOSH_Drivers/Src/WriteToFlash.c \
  Drivers/SwiftOSH_Drivers/Src/RTC_Swift.c \
  Drivers/SwiftOSH_Drivers/Src/AudioCodec.c \
  Drivers/SwiftOSH_Drivers/Src/SwiftSettings.c \
  Drivers/SwiftOSH_Drivers/Src/AudioFiles.c \
  Drivers/SwiftOSH_Drivers/Src/FlashLogging.c \
  Drivers/SwiftOSH_Drivers/Src/StatusLED.c \
  Drivers/SwiftOSH_Drivers/Src/LPModes.c \
  Core/Src/bsp_driver_sd.c \
  Core/Src/sd_diskio.c \
  Core/Src/fatfs.c \
  Middlewares/Third_Party/FatFs/src/ff.c \
  Middlewares/Third_Party/FatFs/src/diskio.c \
  Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
  Middlewares/Third_Party/FatFs/src/option/syscall.c \
  Middlewares/Third_Party/FatFs/src/option/unicode.c \
  Core/Src/syscalls.c \
  Core/Src/swift_defaults.c

# HAL sources (add only what's needed)
C_SOURCES += \
  $(HAL_DIR)/Src/stm32u5xx_hal.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_cortex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_rcc.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_rcc_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_pwr.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_pwr_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_gpio.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_exti.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_dma.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_dma_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_gpdma.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_adc.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_adc_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_i2c.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_i2c_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_lptim.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_pcd.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_pcd_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_rtc.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_rtc_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_sai.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_sd.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_sd_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_tim.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_tim_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_flash.c \
  $(HAL_DIR)/Src/stm32u5xx_hal_flash_ex.c \
  $(HAL_DIR)/Src/stm32u5xx_ll_sdmmc.c \
  $(HAL_DIR)/Src/stm32u5xx_ll_usb.c

# FreeRTOS sources
C_SOURCES += \
  $(FREERTOS_DIR)/croutine.c \
  $(FREERTOS_DIR)/event_groups.c \
  $(FREERTOS_DIR)/list.c \
  $(FREERTOS_DIR)/queue.c \
  $(FREERTOS_DIR)/stream_buffer.c \
  $(FREERTOS_DIR)/tasks.c \
  $(FREERTOS_DIR)/timers.c \
  $(FREERTOS_DIR)/portable/MemMang/heap_4.c \
  $(FREERTOS_DIR)/portable/GCC/ARM_CM33_NTZ/non_secure/port.c \
  $(FREERTOS_DIR)/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c \
  $(FREERTOS_DIR)/CMSIS_RTOS/cmsis_os.c

# USB Device Library sources
C_SOURCES += \
  $(USB_LIB_DIR)/Core/Src/usbd_core.c \
  $(USB_LIB_DIR)/Core/Src/usbd_ctlreq.c \
  $(USB_LIB_DIR)/Core/Src/usbd_ioreq.c \
  $(USB_LIB_DIR)/Class/CustomHID/Src/usbd_customhid.c

# ---- ASM sources ----
ASM_SOURCES = Core/Startup/startup_stm32u545retxq.s

# ---- Linker script ----
LDSCRIPT = STM32U545RETXQ_FLASH.ld

# ---- Compiler flags ----
CFLAGS = $(MCU) $(DEFS) $(INCLUDES) -Wall -fdata-sections -ffunction-sections -std=gnu11
ifdef DEBUG
  CFLAGS += -g -gdwarf-2 -O0
else
  CFLAGS += -Os
endif

ASFLAGS = $(MCU) -Wall -fdata-sections -ffunction-sections

# ---- Linker flags ----
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) \
  -lc -lm -lnosys \
  -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
  -Wl,--gc-sections

# ---- Build rules ----
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).bin
	STM32_Programmer_CLI -c port=SWD -w $< 0x08000000 -v -rst

.PHONY: all clean flash

-include $(wildcard $(BUILD_DIR)/*.d)
