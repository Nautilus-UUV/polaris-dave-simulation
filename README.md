# Polaris DAVE Simulation

ROS2 workspace for Polaris underwater glider simulation in DAVE (Aquatic Robotic Simulator).

## Overview

This workspace contains ROS2 nodes for controlling the Polaris glider's buoyancy system in simulation and preparation for hardware integration.

**Working simulation script:**
- `dive_test` - Simple dive to target depth using Gazebo vertical propeller to simulate bladder

**Future hardware integration:**
- `bladder_control/depth_control_node` - Cascaded PID controller for BCU hardware (motor RPM control)

**Background:** Using custom DAVE fork with Polaris glider model: https://github.com/Nautilus-UUV/dave

## Setup

### Prerequisites

1. **Install ROS2 Jazzy**:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-rclpy ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs
```

2. **Install colcon build tool:**
```bash
sudo apt install python3-colcon-common-extensions
```

3. **Install Python dependencies:**
```bash
pip install -r requirements.txt
```

4. **Install DAVE** - Our custom fork at https://github.com/Nautilus-UUV/dave

5. **Set up shell aliases** (add to `~/.bashrc_aliases`):
```bash
alias jazzy='source /opt/ros/jazzy/setup.bash'
alias harmonic='source /opt/gazebo/install/setup.bash && export PYTHONPATH=$PYTHONPATH:/opt/gazebo/install/lib/python'
alias dave='jazzy && harmonic && source ~/dave_ws/install/setup.bash'
```

### Build this workspace

```bash
cd /home/loco/nautilus/polaris-dave-simulation
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Usage - DAVE Simulation

### 1. Start DAVE simulation

Terminal 1:
```bash
dave
source run.sh
```

This launches Gazebo with the Polaris glider in ocean waves at 1m depth.

### 2. Run dive test

Terminal 2:
```bash
jazzy && harmonic
source install/setup.bash
ros2 run py_pkg dive_test
```

The glider will dive to the target depth (2m by default), stop the bladder when reached, and shutdown when equilibrium is achieved.

## How it works

### dive_test (Current Working Script)

**Purpose:** Test basic buoyancy control in DAVE Gazebo simulation

**Control method:** Direct thrust commands to Gazebo bladder joint
- Subscribes to: `/model/glider_nautilus/sea_pressure` (FluidPressure)
- Commands: `gz topic -t /model/glider_nautilus/joint/bladder_joint/cmd_thrust`

**Parameters** (edit in `src/py_pkg/py_pkg/dive_test.py`):
- `target_depth` - Depth to reach (meters)
- `initial_bladder` - Starting thrust value

**Physics:**
- Pressure to depth: `depth = (pressure - std_pressure) / k`
- `std_pressure = 101.325 kPa`, `k = 9.80638 kPa/m`

### bladder_control/depth_control_node (Future Hardware)

**Purpose:** Control hardware BCU with motor RPM commands

**Control method:** Cascaded PID control (depth → velocity → acceleration → motor RPM)
- For use with actual hardware bladder control unit
- Tested on prototype, ready for when DAVE simulation supports BCU

**Topics:**
- Subscribes to: `/sensor/pressure_ext` (Float64)
- Subscribes to: `/target_depth` (Float64)
- Publishes to: `/BCU_controller/RPM` (Int32)