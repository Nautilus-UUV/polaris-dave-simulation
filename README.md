# Polaris DAVE Simulation

Repository for Polaris scripts to simulate sensors and control nodes.

- [dive_test.py](https://github.com/Nautilus-UUV/polaris-dave-simulation/blob/main/src/py_pkg/py_pkg/dive_test.py), example script to test dave setup.
- [bladder_control_node](https://github.com/Nautilus-UUV/polaris-dave-simulation/tree/main/src/py_pkg/py_pkg/bladder_control_node), example node setup for the 2025 divetest. Fully integrated nodes setup can be found at [polaris-ros2-divetest](https://github.com/Nautilus-UUV/polaris-ros2-divetest).

## Current Status

- [Issues](https://github.com/Nautilus-UUV/polaris-dave-simulation/issues)

## Setup

### Prerequisites

- **[Install DAVE](https://github.com/nautilus-uuv/dave?tab=readme-ov-file#installation)**, our custom fork containing the Polaris model.

> [!TIP]
> **Set up python dependencies**, add to `requirements.txt`:
> ```bash
> pip install -r requirements.txt
> ```

> [!TIP]
> **Set up shell aliases**, add to `~/.bashrc_aliases`:
> ```bash
> alias jazzy='source /opt/ros/jazzy/setup.bash'
> alias harmonic='source /opt/gazebo/install/setup.bash && export PYTHONPATH=$PYTHONPATH:/opt/gazebo/install/lib/python'
> alias dave='jazzy && harmonic && source ~/dave_ws/install/setup.bash'
> ```

### Build this workspace

```bash
cd /home/loco/nautilus/polaris-dave-simulation
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Usage Example

Working example with Polaris custom dave code at [this commit](https://github.com/Nautilus-UUV/dave/tree/bf32034cadaf7b1ae5f4d1979e9b9d3466bc61ea).

### 1. Start DAVE simulation

Terminal 1:
```bash
source /opt/ros/jazzy/setup.bash
source /opt/gazebo/install/setup.bash && export PYTHONPATH=$PYTHONPATH:/opt/gazebo/install/lib/python
source ~/dave_ws/install/setup.bash
source run.sh
```

### 2. Run dive test

Terminal 2:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run py_pkg dive_test
```

## Development

### Workspace Structure

```
polaris-dave-simulation/
├── src/
│   └── py_pkg/
│       ├── py_pkg/
│       │   ├── dive_test.py
│       │   └── bladder_control_node/
│       ├── package.xml
│       └── setup.py
├── requirements.txt
└── run.sh
```

### Creating New Nodes

When adding new ROS2 nodes to this package, follow the structure of the existing `bladder_control_node`:

1. Create a new directory under `src/py_pkg/py_pkg/` for your node
2. Implement your node class inheriting from `rclpy.node.Node`
3. Add the entry point in `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'your_node_name = py_pkg.your_node_directory.main:main',
       ],
   }
   ```
4. Rebuild the workspace with `colcon build`

### Development Workflow

After making changes to the code:

```bash
# Navigate to workspace root
cd ~/nautilus/polaris-dave-simulation

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Rebuild the package
colcon build --symlink-install --packages-select py_pkg

# Source the updated workspace
source install/setup.bash
```

### Debugging

Some useful debugging commands,

- To run nodes with additional debugging output:

```bash
ros2 run py_pkg your_node --ros-args --log-level debug
```

- To inspect active topics and nodes:
```bash
# List all topics
ros2 topic list

# Echo a specific topic
ros2 topic echo /your/topic/name

# List all nodes
ros2 node list

# Get info about a node
ros2 node info /your_node_name
```