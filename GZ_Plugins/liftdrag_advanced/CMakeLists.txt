cmake_minimum_required(VERSION 3.10)
project(LiftDragAdvanced1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(Protobuf REQUIRED)

# Find custom messages directory
# First check the local GZ_Msgs directory, then allow override via environment variable
find_path(CUSTOM_MSGS_DIR
  NAMES gz/msgs/thermal.pb.h
  PATHS
    $ENV{IT_CUSTOM_MSG}
    ${CMAKE_CURRENT_SOURCE_DIR}/../GZ_Msgs/build/msgs-nsgs_genmsg
    ${CMAKE_CURRENT_SOURCE_DIR}/../../GZ_Msgs/build/msgs-nsgs_genmsg
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../GZ_Msgs/build/msgs-nsgs_genmsg
    /usr/local/include
    /usr/include
  NO_DEFAULT_PATH
)

if(NOT CUSTOM_MSGS_DIR)
  message(FATAL_ERROR
    "Could not find custom messages directory containing thermal.pb.h.\n"
    "Please either:\n"
    "1. Build the messages in GZ_Msgs directory: cd ../GZ_Msgs && mkdir build && cd build && cmake .. && make\n"
    "2. Set IT_CUSTOM_MSG environment variable: export IT_CUSTOM_MSG=/path/to/messages\n"
    "3. Pass it to cmake: cmake -DCUSTOM_MSGS_DIR=/path/to/messages\n"
    "Expected structure: GZ_Msgs/build/msgs-nsgs_genmsg/gz/msgs/thermal.pb.h")
endif()

message(STATUS "Found custom messages directory: ${CUSTOM_MSGS_DIR}")

# Add plugin + your custom Protobuf source
add_library(LiftDragAdvanced1 SHARED
  main.cc
  ${CUSTOM_MSGS_DIR}/gz/msgs/thermal.pb.cc
)

# Include directories
target_include_directories(LiftDragAdvanced1 PRIVATE
  ${Protobuf_INCLUDE_DIRS}
  ${CUSTOM_MSGS_DIR}  # for thermal.pb.h
)

# Link required libraries
target_link_libraries(LiftDragAdvanced1
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  ${Protobuf_LIBRARIES}
)

# Install
install(TARGETS LiftDragAdvanced1
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

