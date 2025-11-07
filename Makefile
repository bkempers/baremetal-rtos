CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m7 -mthumb -nostdlib
CPPFLAGS=-DSTM32H7S3xx \
				 -IDrivers/CMSIS/Device/ST/STM32H7RSxx/Include \
				 -IDrivers/CMSIS/Core/Include \
				 -O0 -g3 -Wall

# STM32CubeProgrammer path
STM32_PROG = /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI

LINKER_FILE=linker_script.ld
LDFLAGS=-T $(LINKER_FILE) \
		--specs=nosys.specs \
        -Wl,--gc-sections

BINARY = blink.elf

all: $(BINARY)

$(BINARY): main.o startup_stm32h7s3l8hx.o system_stm32h7rsxx.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $(BINARY)

main.o: main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) main.c -c

startup_stm32h7s3l8hx.o: startup_stm32h7s3l8hx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) startup_stm32h7s3l8hx.c -c

system_stm32h7rsxx.o: Drivers/CMSIS/Device/ST/STM32H7RSxx/Source/Templates/system_stm32h7rsxx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) Drivers/CMSIS/Device/ST/STM32H7RSxx/Source/Templates/system_stm32h7rsxx.c -c

.PHONY: clean
clean:
	rm -f *.o *.elf

# Erase flash
erase:
	$(STM32_PROG) -c port=SWD -e all

gdb-server:
	$(STM32_PROG) -c port=SWD -gdb 3333

flash: $(BINARY)
	arm-none-eabi-size $(BINARY)
	$(STM32_PROG) -c port=SWD -w $(BINARY) -v -rst -q
