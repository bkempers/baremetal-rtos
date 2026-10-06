/* linker_script.ld for STM32H7RS */

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(DTCM) + LENGTH(DTCM); /* end of "DTCM" Ram type memory */

_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400;     /* required amount of stack */

__FLASH_BEGIN  = 0x08000000;
__FLASH_SIZE   = 0x00010000;

__RAM_BEGIN    = 0x24000000;
__RAM_SIZE     = 0x71C00;
__RAM_NONCACHEABLEBUFFER_SIZE = 0x400;

/*TODO: switch to this after boot+app external flash can be used */
/* __FLASH_BEGIN  = 0x70000000;  /* External OctoSPI flash */
/* __FLASH_SIZE   = 0x02000000;  /* 32MB (256MBit) */

/* Memory definitions */
MEMORY
{
    RAM       (xrw) : ORIGIN = __RAM_BEGIN,    LENGTH = __RAM_SIZE
    RAM_NONCACHEABLEBUFFER (xrw) : ORIGIN = __RAM_BEGIN + __RAM_SIZE,  LENGTH = __RAM_NONCACHEABLEBUFFER_SIZE

    ITCM      (xrw) : ORIGIN = 0x00000000,    LENGTH = 0x00010000
    DTCM       (rw) : ORIGIN = 0x20000000,    LENGTH = 0x00010000
    SRAMAHB   (rw)  : ORIGIN = 0x30000000,  LENGTH = 0x00008000
    BKPSRAM   (rw)  : ORIGIN = 0x38800000,  LENGTH = 0x00001000

    FLASH     (xrw) : ORIGIN = __FLASH_BEGIN, LENGTH = __FLASH_SIZE
}

/* Sections */
SECTIONS
{
      /* The startup code into "FLASH" Rom type memory */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */
        . = ALIGN(4);
    } >FLASH

    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.text)                    /* .text sections (code) */
        *(.text*)                   /* .text* sections (code) */
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
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH
    
    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } > FLASH
    
    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } > FLASH
    
    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
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
    } > RAM AT> FLASH

    /* Uninitialized data section */
    . = ALIGN(4);
    .bss :
    {
        /* This is used by the startup in order to initialize the .bss section */
        _sbss = .;                  /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;                  /* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } > RAM

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
    } > DTCM

    /* Define heap end (grows toward stack) */
    _heap_end = ORIGIN(RAM) + LENGTH(RAM) - _Min_Stack_Size;

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
