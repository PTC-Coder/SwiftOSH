/**
  ******************************************************************************
  * @file    STM32U545RETXQ_FLASH.ld
  * @brief   GCC linker script for STM32U545RET6Q
  *          512 KB Flash
  *          SRAM1 = 192 KB @ 0x20000000
  *          SRAM2 =  64 KB @ 0x20030000 (contiguous with SRAM1)
  *          SRAM4 =  16 KB @ 0x28000000 (SmartRun domain, NOT contiguous!)
  *          Total contiguous RAM = 256 KB (SRAM1 + SRAM2)
  ******************************************************************************
  */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x2000;
_Min_Stack_Size = 0x2000;

MEMORY
{
  FLASH    (rx)  : ORIGIN = 0x08000000, LENGTH = 504K   /* Code + rodata: pages 0-62 */
  SETTINGS (rx)  : ORIGIN = 0x0807E000, LENGTH = 8K     /* Page 63: Swift config defaults */
  RAM      (rw)  : ORIGIN = 0x20000000, LENGTH = 256K   /* SRAM1 + SRAM2 contiguous */
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Swift configuration defaults — placed at page 63 (0x0807E000).
     Linker will NOT erase/overwrite this page unless the section has content.
     The config utility and SwiftSettings.c read from these fixed addresses. */
  .swift_settings 0x0807E000 :
  {
    KEEP(*(.swift_settings))
    KEEP(*(.swift_settings.*))
  } >SETTINGS

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
