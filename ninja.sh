#!/bin/bash

run=false
clean=false
debug=false
install=false
preset=rpi

# Parse args
for arg in "$@"; do
    case $arg in
        -r|--run) run=true ;;
        -c|--clean) clean=true ;;
        -d|--debug) debug=true ;;
        -i|--install) install=true ;;
    esac
done

# Clean: Remove the whole directory so the next run re-configures
if $clean; then
    echo "Removing build directory..."
    rm -rf build
    exit 0
fi

# Configuration: Run preset if build folder is missing
if [ ! -d "build" ]; then
    echo "Build directory not found. Configuring with linux preset..."
    cmake --preset=$preset
fi

# Build
cmake --build build -j$(nproc)

# Run normally
if $run; then
    sudo ./build/mover
fi

if $install; then
    sudo cmake --install build
fi

# Debug: Start GDB and run the program immediately
if $debug; then
    echo "Starting GDB..."
    sudo gdb -ex run ./build/mover
fi


# echo "set debuginfod enabled on" >> ~/.gdbinit
# set debuginfod enabled off