#ifndef SHELL_COMMAND_H
#define SHELL_COMMAND_H

typedef struct {
    const char *name;
    int (*handler)(int argc, char **argv);
    const char *help;
} shell_command_t;

// Macro to register commands
#define SHELL_COMMAND_REGISTER(cmd_name, func, help_text) \
    static const shell_command_t __cmd_##cmd_name \
    __attribute__((section(".shell_commands"), used)) = { \
        .name = #cmd_name, \
        .handler = func, \
        .help = help_text \
    };

// Symbols provided by linker script
extern shell_command_t __start_shell_commands;
extern shell_command_t __stop_shell_commands;

#endif
