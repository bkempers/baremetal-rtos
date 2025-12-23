// syscalls.c - Newlib syscall implementations for bare metal STM32

#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

// locals
#include "console.h"
#include "stm32h7rs_hal_usart.h"

extern USART_Handle usart3;

// Defined by linker script
extern uint32_t _heap_start; // Start of heap
extern uint32_t _heap_end;   // End of heap (or stack start)

// Current heap pointer
static uint8_t *heap_ptr = (uint8_t *) &_heap_start;

/**
 * _sbrk - Increase program data space
 * Used by malloc to allocate heap memory
 *
 * @param incr: Number of bytes to allocate
 * @return: Pointer to start of new space, or -1 on error
 */
void *_sbrk(int incr)
{
    uint8_t *prev_heap_ptr = heap_ptr;

    // Check if we have enough space
    if (heap_ptr + incr > (uint8_t *) &_heap_end) {
        // Out of memory!
        errno = ENOMEM;
        return (void *) -1;
    }

    heap_ptr += incr;
    return (void *) prev_heap_ptr;
}

/**
 * _write - Write to a file descriptor
 * fd 1 = stdout, fd 2 = stderr
 *
 * This is called by printf, puts, etc.
 */
int _write(int file, char *ptr, int len)
{
    (void) file;
    //(void) ptr;

    return Console_Write(ptr, len);
}

/**
 * _read - Read from a file descriptor
 * fd 0 = stdin
 */
int _read(int file, char *ptr, int len)
{
    (void) file;
    (void) ptr;
    (void) len;

    return 0;
}

/**
 * _close - Close a file descriptor
 */
int _close(int file)
{
    (void) file;
    return -1; // No files to close
}

/**
 * _fstat - Status of an open file
 */
int _fstat(int file, struct stat *st)
{
    (void) file;

    // Pretend it's a character device
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * _isatty - Query whether output stream is a terminal
 */
int _isatty(int file)
{
    (void) file;
    return 1; // Yes, we're a "terminal"
}

/**
 * _lseek - Set position in a file
 */
int _lseek(int file, int ptr, int dir)
{
    (void) file;
    (void) ptr;
    (void) dir;
    return 0;
}

/**
 * _exit - Exit program
 * Called when main() returns or exit() is called
 */
void _exit(int status)
{
    (void) status;

    // Infinite loop - program ends here
    while (1) {
        __asm__("nop");
    }
}

/**
 * _kill - Send a signal to a process
 */
int _kill(int pid, int sig)
{
    (void) pid;
    (void) sig;
    errno = EINVAL;
    return -1;
}

/**
 * _getpid - Get process ID
 */
int _getpid(void)
{
    return 1; // We're the only "process"
}
