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
    info            Show microcontroller info
    format          Auto-format code
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

# Flash to target
flash() {
    print_msg "Building before flash..."
    build_project $1
    
    print_msg "Flashing to target..."
    cmake --build ${BUILD_DIR} --target flash || { print_error "Flash failed"; exit 1; }
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

# Show memory usage
show_size() {
    if [ -f "${BUILD_DIR}/Application/stm_rtos.elf" ]; then
        print_info "Memory usage:"
        ./scripts/print_size.sh ${BUILD_DIR}/Application/stm_rtos.elf
        #arm-none-eabi-size ${BUILD_DIR}/Application/stm_rtos.elf
    fi
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
    info)
        show_info
        ;;
    format)
        format_code
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
