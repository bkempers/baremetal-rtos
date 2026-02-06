#!/bin/bash

ELF_FILE=$1
PROJECT_NAME=$2
FLASH_SIZE=${3:-33554432}  # Default 32MB external flash
RAM_SIZE=${4:-634880}      # Default 620KB

if [ ! -f "$ELF_FILE" ]; then
    echo "Error: ELF file not found: $ELF_FILE"
    exit 1
fi

# Get sizes
SIZE_OUTPUT=$(arm-none-eabi-size -A -d "$ELF_FILE")
TEXT=$(echo "$SIZE_OUTPUT" | awk '/\.text/ {sum+=$2} END {print sum}')
RODATA=$(echo "$SIZE_OUTPUT" | awk '/\.rodata/ {sum+=$2} END {print sum}')
SHELL_COMMANDS=$(echo "$SIZE_OUTPUT" | awk '/\.shell_commands/ {sum+=$2} END {print sum}')
DATA=$(echo "$SIZE_OUTPUT" | awk '/\.data/ {sum+=$2} END {print sum}')
BSS=$(echo "$SIZE_OUTPUT" | awk '/\.bss/ {sum+=$2} END {print sum}')

TEXT=${TEXT:-0}
RODATA=${RODATA:-0}
SHELL_COMMANDS=${SHELL_COMMANDS:-0}
DATA=${DATA:-0}
BSS=${BSS:-0}

FLASH_USED=$((TEXT + RODATA + DATA + SHELL_COMMANDS))
RAM_USED=$((DATA + BSS))

FLASH_PERCENT=$(awk "BEGIN {printf \"%.1f\", $FLASH_USED*100/$FLASH_SIZE}")
RAM_PERCENT=$(awk "BEGIN {printf \"%.1f\", $RAM_USED*100/$RAM_SIZE}")

# Format with appropriate units based on size
if [ $FLASH_SIZE -ge 1048576 ]; then
    # Use MB for large flash (external)
    FLASH_USED_DISPLAY=$(awk "BEGIN {printf \"%.2f MB\", $FLASH_USED/1024/1024}")
    FLASH_SIZE_DISPLAY=$(awk "BEGIN {printf \"%.0f MB\", $FLASH_SIZE/1024/1024}")
else
    # Use KB for small flash (internal/boot)
    FLASH_USED_DISPLAY=$(awk "BEGIN {printf \"%.2f KB\", $FLASH_USED/1024}")
    FLASH_SIZE_DISPLAY=$(awk "BEGIN {printf \"%.0f KB\", $FLASH_SIZE/1024}")
fi

echo "========================================"
echo "$PROJECT_NAME Memory Usage:"
echo "========================================"
printf "Flash: %d bytes (%s / %s) [%s%% used]\n" \
    $FLASH_USED "$FLASH_USED_DISPLAY" "$FLASH_SIZE_DISPLAY" "$FLASH_PERCENT"
printf "  .text:    %7d bytes (%.2f KB)\n" $TEXT $(awk "BEGIN {printf \"%.2f\", $TEXT/1024}")
printf "  .rodata:  %7d bytes (%.2f KB)\n" $RODATA $(awk "BEGIN {printf \"%.2f\", $RODATA/1024}")
[ $SHELL_COMMANDS -gt 0 ] && printf "  .sh_cmds: %7d bytes (%.2f KB)\n" $SHELL_COMMANDS $(awk "BEGIN {printf \"%.2f\", $SHELL_COMMANDS/1024}")
printf "  .data:    %7d bytes (%.2f KB)\n" $DATA $(awk "BEGIN {printf \"%.2f\", $DATA/1024}")
echo ""
printf "RAM:   %d bytes (%.2f KB / %.0f KB) [%s%% used]\n" \
    $RAM_USED $(awk "BEGIN {printf \"%.2f\", $RAM_USED/1024}") \
    $(awk "BEGIN {printf \"%.0f\", $RAM_SIZE/1024}") $RAM_PERCENT
printf "  .data:    %7d bytes (%.2f KB)\n" $DATA $(awk "BEGIN {printf \"%.2f\", $DATA/1024}")
printf "  .bss:     %7d bytes (%.2f KB)\n" $BSS $(awk "BEGIN {printf \"%.2f\", $BSS/1024}")
echo "========================================"
