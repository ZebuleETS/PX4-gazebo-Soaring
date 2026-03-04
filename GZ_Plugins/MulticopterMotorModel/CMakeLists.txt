cmake_minimum_required(VERSION 3.10)
project(MulticopterMotorModel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

# Add plugin library
add_library(gz-sim-multicopter-motor-model-system SHARED
  MulticopterMotorModel.cc
)

# Link required libraries
target_link_libraries(gz-sim-multicopter-motor-model-system
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  gz-common5::gz-common5
)

# Install
install(TARGETS gz-sim-multicopter-motor-model-system
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
