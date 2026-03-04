# Advanced LiftDrag Plugin for Gazebo

This plugin provides advanced aerodynamic simulation with thermal updraft support for fixed-wing aircraft in Gazebo.

## Features

- **Advanced Aerodynamics**: Realistic lift, drag, and moment calculations
- **Thermal Updraft Simulation**: Integration with thermal detection systems
- **Control Surface Support**: Configurable control surfaces (ailerons, elevators, rudder)
- **Stall Modeling**: Realistic stall behavior with configurable coefficients
- **Wind Integration**: Support for environmental wind effects

## Dependencies

### Required Gazebo Packages
- `gz-sim8` (Gazebo Sim)
- `gz-msgs10` (Gazebo Messages)
- `gz-transport13` (Gazebo Transport)
- `gz-math7` (Gazebo Math)
- `Protobuf` (Protocol Buffers)

### Custom Messages
This plugin requires custom thermal messages. You need to build the thermal message definitions first.

## Building the Plugin

### Quick Start (Recommended)

Use the provided script for easy setup:

```bash
# Navigate to the PX4-Autopilot-FV root directory
cd /path/to/PX4-Autopilot-FV

# Build everything and set up environment (default)
./liftdrag_plugin.sh

# Or explicitly build
./liftdrag_plugin.sh build

# Or only set up environment (if already built)
./liftdrag_plugin.sh setup
```

### Manual Build Process

If you prefer to build manually:

#### 1. Build Custom Messages (Required)

```bash
# Navigate to the GZ_Msgs directory
cd /path/to/PX4-Autopilot-FV/Tools/simulation/gz/GZ_Msgs

# Build the messages
mkdir -p build && cd build
cmake .. && make
```

#### 2. Build the Plugin

```bash
# Navigate to the plugin directory
cd /path/to/PX4-Autopilot-FV/Tools/simulation/gz/GZ_Plugins/liftdrag

# The plugin will automatically find the messages in ../GZ_Msgs/build/msgs-nsgs_genmsg
# If you need to override the path, you can use:

# Method 1: Environment variable
# export IT_CUSTOM_MSG=/path/to/your/custom_messages/build/msgs-nsgs_genmsg

# Method 2: CMake variable
# cmake -DCUSTOM_MSGS_DIR=/path/to/your/custom_messages/build/msgs-nsgs_genmsg ..

# Build the plugin
mkdir -p build && cd build
cmake ..
make
```

### 3. Setup Environment

#### Option 1: Use Setup Script (Recommended)

```bash
# Navigate to PX4-Autopilot-FV root directory
cd /path/to/PX4-Autopilot-FV

# Setup environment variables
source liftdrag_plugin.sh setup

# Make permanent (optional)
echo 'source $(pwd)/liftdrag_plugin.sh setup' >> ~/.bashrc
```

#### Option 2: Manual Environment Setup

```bash
# Set custom messages path
export IT_CUSTOM_MSG=/path/to/PX4-Autopilot-FV/Tools/simulation/gz/GZ_Msgs/build/msgs-nsgs_genmsg

# Set plugin path
export GZ_SIM_PLUGIN_PATH=/path/to/PX4-Autopilot-FV/Tools/simulation/gz/GZ_Plugins/liftdrag/build:$GZ_SIM_PLUGIN_PATH

# Set library path (backup)
export LD_LIBRARY_PATH=/path/to/PX4-Autopilot-FV/Tools/simulation/gz/GZ_Plugins/liftdrag/build:$LD_LIBRARY_PATH
```

#### Option 3: System Installation

```bash
# Install to system directory (alternative to environment variables)
sudo cp libLiftDrag.so /usr/lib/x86_64-linux-gnu/gz-sim-6/plugins/
```

## Usage in SDF

Add the plugin to your model's SDF file:

```xml
<plugin filename="libLiftDrag.so" name="gz::sim::systems::LiftDrag">
  <!-- Basic aerodynamic parameters -->
  <link_name>base_link</link_name>
  <air_density>1.2041</air_density>
  <area>0.34</area>
  <a0>0.0</a0>

  <!-- Lift coefficients -->
  <cla>5.015</cla>
  <alpha_stall>0.3391428111</alpha_stall>
  <cla_stall>-3.85</cla_stall>

  <!-- Drag coefficients -->
  <cda>0.029</cda>
  <cda_stall>-0.9233984055</cda_stall>

  <!-- Moment coefficients -->
  <cma>-0.463966</cma>
  <cma_stall>0</cma_stall>

  <!-- Geometry -->
  <cp>-0.12 0.0 0.0</cp>
  <forward>1 0 0</forward>
  <upward>0 0 1</upward>

  <!-- Control surface (optional) -->
  <control_joint_name>servo_0</control_joint_name>
  <control_joint_rad_to_cl>4.0</control_joint_rad_to_cl>
  <cm_delta>0.0</cm_delta>
</plugin>
```

## Configuration Parameters

### Required Parameters
- `<link_name>`: Name of the link to apply forces to
- `<area>`: Reference area for aerodynamic calculations
- `<cla>`: Lift coefficient slope
- `<cda>`: Drag coefficient slope

### Optional Parameters
- `<air_density>`: Air density (default: 1.2041 kg/m³)
- `<a0>`: Initial angle of attack (default: 0.0)
- `<alpha_stall>`: Stall angle of attack (default: π/2)
- `<cla_stall>`: Lift coefficient slope after stall (default: 0.0)
- `<cda_stall>`: Drag coefficient slope after stall (default: 1.0)
- `<cma>`: Moment coefficient slope (default: 0.0)
- `<cma_stall>`: Moment coefficient slope after stall (default: 0.0)
- `<cp>`: Center of pressure (default: 0 0 0)
- `<forward>`: Forward direction vector (default: 1 0 0)
- `<upward>`: Upward direction vector (default: 0 0 1)
- `<radial_symmetry>`: Whether the shape is radially symmetric (default: false)
- `<control_joint_name>`: Name of control surface joint
- `<control_joint_rad_to_cl>`: Control surface effectiveness (default: 4.0)
- `<cm_delta>`: Moment change per control deflection (default: 0.0)

## Thermal Integration

The plugin automatically subscribes to thermal updraft messages on the topic:
```
/world/default/thermal_updrafts
```

Make sure your thermal detection system publishes messages in the `gz::msgs::ThermalGroup` format.

## Troubleshooting

### Build Issues

1. **"Could not find custom messages directory"**
   - Ensure you've built the custom messages in `../GZ_Msgs/build/msgs-nsgs_genmsg`
   - Check that `thermal.pb.h` exists in `../GZ_Msgs/build/msgs-nsgs_genmsg/gz/msgs/`
   - If using a different location, set `IT_CUSTOM_MSG` environment variable

2. **Missing Gazebo packages**
   - Install required Gazebo packages for your distribution
   - Check package versions match the requirements

3. **Plugin not found at runtime**
   - Install plugin to system directory, or
   - Set `GZ_SIM_PLUGIN_PATH` environment variable

### Runtime Issues

1. **No forces applied**
   - Check that `<link_name>` matches an existing link
   - Verify aerodynamic parameters are reasonable
   - Check for error messages in Gazebo console

2. **Thermal effects not working**
   - Ensure thermal messages are being published
   - Check topic name matches `/world/default/thermal_updrafts`
   - Verify message format is correct

## License

This plugin is based on the Open Source Robotics Foundation's LiftDrag system and is licensed under the Apache License 2.0.
