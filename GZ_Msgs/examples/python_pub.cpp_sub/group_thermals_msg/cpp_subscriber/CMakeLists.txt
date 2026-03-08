cmake_minimum_required(VERSION 3.10)
project(ThermalGroupSubscriber)

# Find required packages
find_package(gz-transport13 REQUIRED)
find_package(Protobuf REQUIRED)

# Path to the GZ_Msgs root (4 levels up from this CMakeLists.txt)
set(GZ_MSGS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
set(GZ_MSGS_GENMSG ${GZ_MSGS_DIR}/build/msgs-nsgs_genmsg)

# Include directories for generated Protobuf files
include_directories(
    ${GZ_MSGS_GENMSG}  # Path to generated Protobuf headers
)

# Add the executable
add_executable(thermal_sub 
    subscriber_thermal_group.cc 
    ${GZ_MSGS_GENMSG}/gz/msgs/thermal.pb.cc  # Generated Protobuf source file
)

# Link libraries
target_link_libraries(thermal_sub 
    gz-transport13::gz-transport13  # Gazebo Transport
    ${Protobuf_LIBRARIES}           # Protobuf library
)

# Include Protobuf headers
target_include_directories(thermal_sub PRIVATE ${Protobuf_INCLUDE_DIRS})

