/* memory.x */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 1024K   /* 1MB of flash memory */
    RAM   : ORIGIN = 0x20000000, LENGTH = 320K    /* 320KB of RAM */
}

/* Provide the entry point for the application */
_stack_start = ORIGIN(RAM) + LENGTH(RAM); /* Stack pointer at the end of RAM */

