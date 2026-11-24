/*
 * stm32f405.ld - linker script for STM32F405RG/VG (1 MiB flash, 128 KiB
 * SRAM + 64 KiB CCM RAM).
 *
 * Flash layout:
 *   0x08000000  sectors 0..10 (896 KiB)  vectors + code + rodata + .data LMA
 *   0x080E0000  sector 11     (128 KiB)  persistent configuration (CLI 'save')
 */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 896K
    CONFIG (r)   : ORIGIN = 0x080E0000, LENGTH = 128K
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    CCM    (rw)  : ORIGIN = 0x10000000, LENGTH = 64K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack = 0x2000; /* 8 KiB reserved for the main stack */

SECTIONS
{
    .isr_vector : ALIGN(4)
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : ALIGN(4)
    {
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        *(.glue_7) *(.glue_7t)
        . = ALIGN(4);
    } > FLASH

    .ARM.exidx : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } > FLASH

    _sidata = LOADADDR(.data);

    .data : ALIGN(4)
    {
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* not cleared at reset: used to pass the DFU request across a reset */
    .noinit (NOLOAD) : ALIGN(4)
    {
        *(.noinit .noinit.*)
        . = ALIGN(4);
    } > RAM

    .stack_guard (NOLOAD) : ALIGN(8)
    {
        . = . + _min_stack;
    } > RAM

    /DISCARD/ : { *(.comment) *(.note*) }
}
