#!/bin/bash
set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project settings
BUILD_DIR=".build"
BUILD_TYPE="Debug"

STM32_PROG="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"

# Print colored message
print_msg() {
    echo -e "${GREEN}[BUILD]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Check if a USB device is an STM32 Nucleo board
is_stm32_device() {
    local device=$1
    
    # Get detailed USB device info for this specific serial device
    local ioreg_data=$(ioreg -r -c IOUSBHostDevice -l | grep -B 20 -A 20 "$device" || true)
    
    if [ -z "$ioreg_data" ]; then
        return 1
    fi
    
    # Look for STMicroelectronics vendor ID (0x483 or decimal 1155)
    # The ioreg output shows it as "idVendor" = 1155 or 0x483
    if echo "$ioreg_data" | grep -E '"idVendor" = (1155|0x483)' > /dev/null 2>&1; then
        return 0
    fi
    
    return 1
}

# Show usage
usage() {
    cat << EOF
Usage: ./build.sh [COMMAND] [OPTIONS]

Commands:
    build           Build the project (default)
    clean           Remove build directory
    rebuild         Clean and build
    flash           Build and flash to target
    erase           Erase target flash memory
    gdb             Start GDB server
    size            Show memory usage
    serial          Connect to STM32 board via serial (default: 115200 baud)
    info            Show microcontroller info
    format          Auto-format code
    check           Run static analyzer
    release         Build in release mode
    help            Show this help message

Options:
    -j, --jobs N    Number of parallel jobs (default: number of cores)
    -v, --verbose   Verbose output

Examples:
    ./build.sh              # Build in debug mode
    ./build.sh build        # Same as above
    ./build.sh flash        # Build and flash
    ./build.sh rebuild      # Clean and build
    ./build.sh release      # Build in release mode
    ./build.sh clean        # Clean build directory
EOF
}

# Configure CMake
configure() {
    local build_type=$1
    print_msg "Configuring CMake (${build_type} mode)..."
    mkdir -p ${BUILD_DIR}
    cd ${BUILD_DIR}
    cmake -DCMAKE_BUILD_TYPE=${build_type} \
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
          .. || { print_error "CMake configuration failed"; exit 1; }
    cd ..

    # Symlink compile_commands.json for LSP
    ln -sf ${BUILD_DIR}/compile_commands.json compile_commands.json
    print_info "compile_commands.json linked for LSP"
}

# Build project
build_project() {
    local jobs=${1:-$(sysctl -n hw.ncpu)}
    
    # Check if CMake is configured
    if [ ! -f "${BUILD_DIR}/Makefile" ] && [ ! -f "${BUILD_DIR}/build.ninja" ]; then
        configure ${BUILD_TYPE}
    fi
    
    print_msg "Building project..."
    cmake --build ${BUILD_DIR} -j${jobs} || { print_error "Build failed"; exit 1; }
    
    print_msg "Build complete!"
    show_size
}

# Clean build directory
clean() {
    print_msg "Cleaning build directory..."
    rm -rf ${BUILD_DIR}
    rm -f compile_commands.json
    print_msg "Clean complete!"
}

# Rebuild (clean + build)
rebuild() {
    clean
    build_project $1
}

# Show memory usage
show_size() {
    if [ -f "${BUILD_DIR}/Application/stm_rtos.elf" ]; then
        print_info "Memory usage:"
        ./scripts/print_size.sh ${BUILD_DIR}/Application/stm_rtos.elf
    fi
}

# Flash to target
flash() {
    print_msg "Flashing to target..."
    cmake --build ${BUILD_DIR} --target flash || { print_error "Flash failed"; exit 1; }
    show_size
    print_msg "Flash complete!"
}

# Erase flash
erase() {
    print_msg "Erasing target flash..."
    cmake --build ${BUILD_DIR} --target erase || { print_error "Erase failed"; exit 1; }
    print_msg "Erase complete!"
}

# Start GDB server
gdb_server() {
    print_msg "Starting GDB server on port 61234..."
    print_info "In another terminal, run: arm-none-eabi-gdb ${BUILD_DIR}/stm_rtos.elf"
    print_info "Then in GDB: target remote :61234"
    cmake --build ${BUILD_DIR} --target gdb-server
}

# Connect to STM32 Nucleo board via serial
connect_serial() {
    local baud_rate=${1:-115200}
    
    print_msg "Searching for STM32 Nucleo boards..."
    
    # Find all tty.usbmodem devices
    local devices=(/dev/tty.usbmodem*)
    local stm32_devices=()
    
    # Check if any devices exist
    if [ ! -e "${devices[0]}" ]; then
        print_error "No USB modem devices found"
        print_info "Make sure your STM32 Nucleo board is connected"
        return 1
    fi
    
    # Filter for STM32 devices
    for device in "${devices[@]}"; do
        local device_name=$(basename "$device")
        if is_stm32_device "$device_name"; then
            stm32_devices+=("$device")
            print_info "Found STM32 device: $device"
        fi
    done
    
    # Check if we found any STM32 devices
    if [ ${#stm32_devices[@]} -eq 0 ]; then
        print_error "No STM32 Nucleo boards found"
        print_info "Found USB modems, but none are STMicroelectronics devices:"
        for device in "${devices[@]}"; do
            echo "  - $device"
        done
        return 1
    fi
    
    local selected_device
    
    # If only one STM32 device, use it automatically
    if [ ${#stm32_devices[@]} -eq 1 ]; then
        selected_device="${stm32_devices[0]}"
        print_msg "Connecting to ${selected_device} at ${baud_rate} baud..."
    else
        # Multiple devices - let user choose
        print_warning "Multiple STM32 devices found:"
        for i in "${!stm32_devices[@]}"; do
            echo "  [$i] ${stm32_devices[$i]}"
        done
        
        read -p "Select device number [0-$((${#stm32_devices[@]}-1))]: " selection
        
        if [[ "$selection" =~ ^[0-9]+$ ]] && [ "$selection" -ge 0 ] && [ "$selection" -lt ${#stm32_devices[@]} ]; then
            selected_device="${stm32_devices[$selection]}"
            print_msg "Connecting to ${selected_device} at ${baud_rate} baud..."
        else
            print_error "Invalid selection"
            return 1
        fi
    fi
    
    # Connect using screen
    print_info "Press Ctrl+A then K to exit screen session"
    print_info "Press Ctrl+A then Ctrl+D to detach (leave running in background)"
    sleep 1
    screen "$selected_device" $baud_rate
}

# Show STM32 Info 
show_info() {
    print_info "Querying connected STM32 microcontroller..."
    ${STM32_PROG} -c port=SWD || {
        print_error "Failed to connect to STM32"
        print_info "Check that:"
        echo "  - ST-Link is connected"
        echo "  - Board is powered"
        echo "  - SWD connections are correct"
        return 1
    }
}

# Clangd format
format_code() {
    # Format all C/C++ files under specific top-level directories
    MODULES=(
        "HAL"
        "Application"
        "RTOS"
    )

    for module in "${MODULES[@]}"; do
        if [ -d "$module" ]; then
            find "$module" -type f \( -name "*.c" -o -name "*.h" \) \
                -exec clang-format -i {} \;
        fi
    done

    echo "Formatting complete!"

}

# cppcheck Static analyzer
check_code() {
    echo "Running cppcheck static analyzer"
    
    cppcheck --quiet -I Application/ -I HAL/ -I rtos/ .
}


# Build in release mode
build_release() {
    BUILD_TYPE="Release"
    print_warning "Building in RELEASE mode"
    clean
    configure ${BUILD_TYPE}
    build_project $1
}

# Parse arguments
COMMAND=${1:-build}
shift || true

JOBS=$(sysctl -n hw.ncpu)
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=true
            set -x
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Execute command
case ${COMMAND} in
    build)
        build_project ${JOBS}
        ;;
    clean)
        clean
        ;;
    rebuild)
        rebuild ${JOBS}
        ;;
    flash)
        flash ${JOBS}
        ;;
    erase)
        erase
        ;;
    gdb)
        gdb_server
        ;;
    size)
        show_size
        ;;
    serial)
        connect_serial 115200
        ;;
    info)
        show_info
        ;;
    format)
        format_code
        ;;
    check)
        check_code
        ;;
    release)
        build_release ${JOBS}
        ;;
    help|--help|-h)
        usage
        ;;
    *)
        print_error "Unknown command: ${COMMAND}"
        usage
        exit 1
        ;;
esac
