/* Entry Point */
ENTRY(Reset_Handler)

/* _Min_Heap_Size = 0x200;      required amount of heap  */
/* _Min_Stack_Size = 0x400;     required amount of stack */

/* Memory definitions */
MEMORY
{
    /* External flash via XSPI2 */
    FLASH (rx)   : ORIGIN = 0x70000000, LENGTH = 256M
    
    /* RAM regions */
    DTCMRAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 64K
    AXISRAM (xrw)   : ORIGIN = 0x24000000, LENGTH = 320K
    RAM (xrw)       : ORIGIN = 0x24000000, LENGTH = 320K  /* Alias for AXISRAM */
    SRAM1 (xrw)     : ORIGIN = 0x24050000, LENGTH = 64K
    SRAM2 (xrw)     : ORIGIN = 0x24060000, LENGTH = 64K
}

/* Highest address of the user mode stack */
_estack = ORIGIN(AXISRAM) + LENGTH(AXISRAM);

/* Heap and stack sizes for application */
_Min_Heap_Size = 0x4000;      /* 16KB heap */
_Min_Stack_Size = 0x8000;     /* 32KB stack */

/* Sections */
SECTIONS
{
    /* Vector table for application at start of external flash */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code (vector table) */
        . = ALIGN(4);
    } >FLASH

    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.text)                    /* .text sections (code) */
        *(.text*)                   /* .text sections (code) */
        *(.glue_7)                  /* glue arm to thumb code */
        *(.glue_7t)                 /* glue thumb to arm code */
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;                 /* define a global symbols at end of code */
    } > FLASH

    /* Constant data goes into FLASH */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)                  /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)                 /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } > FLASH
    
    .ARM :
    {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(4);
    } > FLASH

    .preinit_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } > FLASH
    
    .init_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(4);
    } > FLASH
    
    .fini_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
    } > FLASH

    .shell_commands : {
        . = ALIGN(4);
        PROVIDE(__start_shell_commands = .);
        KEEP(*(.shell_commands))
        PROVIDE(__stop_shell_commands = .);
        . = ALIGN(4);
    } > FLASH

    /* Used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data sections goes into RAM, load LMA copy after code */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;                 /* create a global symbol at data start */
        *(.data)                    /* .data sections */
        *(.data*)                   /* .data* sections */
        *(.RamFunc)                 /* .RamFunc sections */
        *(.RamFunc*)                /* .RamFunc* sections */

        . = ALIGN(4);
        _edata = .;                 /* define a global symbol at data end */
    } > AXISRAM AT> FLASH

    /* Uninitialized data section */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;                  /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;                  /* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } > AXISRAM

    /* User_heap_stack section, used to check that there is enough RAM left */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        _heap_start = .;            /* Heap start for malloc */
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > AXISRAM

    /* Define heap end (grows toward stack) */
    _heap_end = ORIGIN(RAM) + LENGTH(RAM) - _Min_Stack_Size;

    /* Remove information from the standard libraries */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
