#ifndef COMPILER_H
#define COMPILER_H

#if defined(__GNUC__)
#ifndef __weak
#define __weak __attribute__((weak))
#endif
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif
#ifndef __aligned
#define __aligned(x) __attribute__((aligned(x)))
#endif
#ifndef __naked
#define __naked __attribute__((naked))
#endif

#elif defined(__ICCARM__) // IAR compiler
#define __packed     __packed
#define __aligned(x) _Pragma("data_alignment=" #x)
#define __naked      __task

#elif defined(__CC_ARM) // Keil compiler
#define __packed     __packed
#define __aligned(x) __align(x)
#define __naked      __asm

#else
#warning "Unsupported compiler"
#define __weak
#define __packed
#define __aligned(x)
#define __naked
#endif

#define UNUSED(X) (void) (X) /* To avoid gcc/g++ warnings */

#endif // COMPILER_H
