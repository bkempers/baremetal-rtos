#include "console.h"
#include "stm32h7rs_hal_rcc.h"
#include <stdio.h>
#include <string.h>

#include "system.h"

static void system_usage()
{
    PRINT_INFO("System information usage: \r\n \
        - info: Get some basic information \r\n \
        - ver: Get the version information \r\n \
        - clock: Get the clock configuration \r\n \
        - git: Get the Git versioning of build \r\n");
}

static int system_handler(int argc, char **argv)
{
    if (argc < 2) {
        system_usage();
        return 1;
    }

    if (strcmp(argv[1], "info") == 0) {
        PRINT_INFO("STM32H7RS Serial Console");
        return 0;
    }

    if (strcmp(argv[1], "ver") == 0) {
        PRINT_INFO("VERSION: %u.%u.%u", MAJOR_VER, MINOR_VER, PATCH_VER);
        return 0;
    }

    if (strcmp(argv[1], "clock") == 0) {
        PRINT_INFO("CLOCK: %.1f MHz", (SystemCoreClock / 1e6));
        return 0;
    }

    if (strcmp(argv[1], "git") == 0) {
        PRINT_INFO("GIT BRANCH: %s & HASH: %s", GIT_BRANCH, GIT_COMMIT_SHORT);
        return 0;
    }

    if (strlen(*argv) > 2) {
        PRINT_INFO("Unknown system argument %s", argv[1]);
        return 1;
    }

    return 0;
}
SHELL_COMMAND_REGISTER(system, system_handler, "Access system information")
