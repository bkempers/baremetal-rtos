#!/bin/bash

ELF_FILE=$1
FLASH_SIZE=${2:-2097152}  # Default 2MB
RAM_SIZE=${3:-634880}     # Default 620KB

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

FLASH_USED=$((TEXT + RODATA + DATA))
RAM_USED=$((DATA + BSS))

FLASH_PERCENT=$(awk "BEGIN {printf \"%.1f\", $FLASH_USED*100/$FLASH_SIZE}")
RAM_PERCENT=$(awk "BEGIN {printf \"%.1f\", $RAM_USED*100/$RAM_SIZE}")

echo "========================================"
printf "Flash: %d bytes (%.2f KB) [%s%% used]\n" $FLASH_USED $(awk "BEGIN {printf \"%.2f\", $FLASH_USED/1024}") $FLASH_PERCENT
printf "  .text:   %7d bytes (%.2f KB)\n" $TEXT $(awk "BEGIN {printf \"%.2f\", $TEXT/1024}")
printf "  .rodata: %7d bytes (%.2f KB)\n" $RODATA $(awk "BEGIN {printf \"%.2f\", $RODATA/1024}")
printf "  .sh_cmds: %7d bytes (%.2f KB)\n" $SHELL_COMMANDS $(awk "BEGIN {printf \"%.2f\", $SHELL_COMMANDS/1024}")
printf "  .data:   %7d bytes (%.2f KB)\n" $DATA $(awk "BEGIN {printf \"%.2f\", $DATA/1024}")
echo ""
printf "RAM:   %d bytes (%.2f KB) [%s%% used]\n" $RAM_USED $(awk "BEGIN {printf \"%.2f\", $RAM_USED/1024}") $RAM_PERCENT
printf "  .data:   %7d bytes (%.2f KB)\n" $DATA $(awk "BEGIN {printf \"%.2f\", $DATA/1024}")
printf "  .bss:    %7d bytes (%.2f KB)\n" $BSS $(awk "BEGIN {printf \"%.2f\", $BSS/1024}")
echo "========================================"
