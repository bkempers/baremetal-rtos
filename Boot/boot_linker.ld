/* Entry Point */
ENTRY(Reset_Handler)

/* Memory regions */
MEMORY
{
    /* Internal flash */
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 32K
    
    /* RAM regions */
    DTCMRAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 64K
    AXISRAM (xrw)   : ORIGIN = 0x24000000, LENGTH = 64K    /* Only use small part for boot */
}

/* Highest address of the user mode stack */
_estack = ORIGIN(AXISRAM) + LENGTH(AXISRAM);

/* Heap and stack sizes */
_Min_Heap_Size = 0x200;      /* 512 bytes - minimal for boot */
_Min_Stack_Size = 0x400;     /* 1KB - minimal for boot */

/* Sections */
SECTIONS
{
    /* Vector table MUST be first - CPU boots from 0x08000000 */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */
        . = ALIGN(4);
    } >FLASH

    /* Program code */
    .text :
    {
        . = ALIGN(4);
        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.glue_7)         /* glue arm to thumb code */
        *(.glue_7t)        /* glue thumb to arm code */
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;        /* End of code section */
    } >FLASH

    /* Constant data */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } >FLASH

    /* ARM exception handling */
    .ARM.extab :
    {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } >FLASH

    .ARM :
    {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(4);
    } >FLASH

    .preinit_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .init_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .fini_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
    } >FLASH

    /* Used by startup to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data sections - stored in flash, copied to RAM at boot */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;        /* Create a global symbol at data start */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */
        *(.RamFunc)        /* .RamFunc sections */
        *(.RamFunc*)       /* .RamFunc* sections */

        . = ALIGN(4);
        _edata = .;        /* Define a global symbol at data end */
    } >AXISRAM AT> FLASH

    /* Uninitialized data section - zero initialized */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;         /* Used by startup code to init .bss secion */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* Used by startup code */
        __bss_end__ = _ebss;
    } >AXISRAM

    /* User_heap_stack section for boot - minimal */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >AXISRAM

    /* Remove information from the compiler libraries */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* Check that boot fits in 32KB */
ASSERT(_etext < (ORIGIN(FLASH) + LENGTH(FLASH)), "Boot code exceeds 32KB internal flash!")
