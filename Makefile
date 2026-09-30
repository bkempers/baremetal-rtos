# baremetal-rtos Makefile

BUILD_TYPE ?= Debug
BUILD_DIR  := build/$(BUILD_TYPE)
ELF        := $(BUILD_DIR)/application/baremetal_rtos.elf
JOBS       ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu)
SRC_DIRS   := hal kernel application

PORT       ?= $(firstword $(wildcard /dev/tty.usbmodem*))
BAUD       ?= 115200
STM32_PROG ?= /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI

.PHONY: all help build release rebuild clean flash erase gdb size serial info format check

all: build

help: ## Show this help message
	@echo "Usage: make [TARGET] [VAR=value ...]"
	@echo
	@echo "Targets:"
	@awk 'BEGIN {FS = ":.*## "} /^[a-zA-Z_-]+:.*## / {printf "  %-10s %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo
	@echo "Variables:"
	@echo "  BUILD_TYPE Debug or Release (default: $(BUILD_TYPE))"
	@echo "  JOBS       Parallel jobs (default: $(JOBS))"
	@echo "  PORT       Serial device (default: first /dev/tty.usbmodem*)"
	@echo "  BAUD       Serial baud rate (default: $(BAUD))"

# Configure only once per build type
$(BUILD_DIR)/CMakeCache.txt:
	cmake -S . -B $(BUILD_DIR) \
	      -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

build: $(BUILD_DIR)/CMakeCache.txt
	cmake --build $(BUILD_DIR) -j $(JOBS)
	@ln -sf $(BUILD_DIR)/compile_commands.json compile_commands.json

release:
	@$(MAKE) --no-print-directory BUILD_TYPE=Release build

rebuild:
	@$(MAKE) --no-print-directory clean
	@$(MAKE) --no-print-directory build

clean:
	rm -rf build compile_commands.json

flash: build
	cmake --build $(BUILD_DIR) --target flash

erase: $(BUILD_DIR)/CMakeCache.txt
	cmake --build $(BUILD_DIR) --target erase

gdb: $(BUILD_DIR)/CMakeCache.txt
	@echo "Other terminal: arm-none-eabi-gdb $(ELF) -ex 'target remote :61234'"
	cmake --build $(BUILD_DIR) --target gdb-server

serial:
	@test -n "$(PORT)" || { echo "No /dev/tty.usbmodem* found. Set PORT=..."; exit 1; }
	screen $(PORT) $(BAUD)

info:
	$(STM32_PROG) -c port=SWD

format:
	find $(SRC_DIRS) -type f \( -name '*.c' -o -name '*.h' \) -exec clang-format -i {} +

check:
	cppcheck --quiet $(addprefix -I ,$(SRC_DIRS)) .
