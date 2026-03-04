#!/bin/bash

# Define the build directory
BUILD_DIR="build"

# Clean the previous build
if [ -d "$BUILD_DIR" ]; then
    echo "Cleaning up old build directory..."
    rm -rf "$BUILD_DIR"
fi

# Create the build directory
echo "Creating build directory..."
mkdir "$BUILD_DIR" && cd "$BUILD_DIR"

# Run CMake
echo "Configuring with CMake..."
cmake ..

# Build the project
echo "Building the project..."
make

# Return to the root directory
cd ..

# Run Gazebo Sim with your plugin
#echo "Running Gazebo Sim with the plugin..."
#gz sim world_updraftplugin.sdf