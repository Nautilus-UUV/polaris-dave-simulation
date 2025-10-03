# polaris-dave-simulation

This repository contains simulation assets and configurations for the Polaris Unmanned Underwater Vehicle (UUV) within the DAVE Aquatic Virtual Environment (DAVE) simulation framework.

**Purpose:**

This project aims to provide a realistic and configurable simulation environment for testing, developing, and validating algorithms and control strategies for the Polaris UUV. It leverages the capabilities of DAVE for physics-based simulation, sensor modeling, and integration with ROS 2.

**Installation:**

To use this simulation, you need to have our custom DAVE fork: https://github.com/Nautilus-UUV/dave

**Quick Start:**

Source dave in one terminal:
```bash
alias jazzy='source /opt/ros/jazzy/setup.bash'
alias harmonic='source /opt/gazebo/install/setup.bash && export PYTHONPATH=$PYTHONPATH:/opt/gazebo/install/lib/python'
alias dave='jazzy && harmonic && source ~/dave_ws/install/setup.bash'
dave
```

then start the gazebo simulation by running:

```bash
source run.sh
```

In another terminal, source dave and activate the python venv, then run the desired .py script.

**Further Documentation Needed.**
