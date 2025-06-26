# Custom DWA Planner – Setup & Usage Guide

This document explains how to prepare a fresh Ubuntu 22.04 machine for running the `custom_dwa_planner` package in a TurtleBot3 Gazebo simulation, with optional integration into the Navigation2 (Nav2) stack.

Working Videos Link: [Working Videos](https://drive.google.com/drive/folders/1uTWd6ZoWY1b7fFA6q_QhItbBTVSVw3P6?usp=sharing)

---

## 1. Prerequisites

| Requirement | Version |
|-------------|---------|
| Operating System | **Ubuntu 22.04** |
| ROS 2 distro | **Humble Hawksbill** |
| Simulator | **Gazebo Classic (gazebo11)** |
| (Optional) Nav2 stack | `ros-humble-nav2*` Debian packages or source build |

Install ROS 2 Humble by following the official guide if it is not already present.

```bash
# Make sure you have sourced ROS 2 after installation
source /opt/ros/humble/setup.bash
```

---

## 2. Create / prepare the workspace

```bash
mkdir -p ~/final_ws/src
cd ~/final_ws/src
```

### 2.1. Clone TurtleBot3 repositories

```bash
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

### 2.2. Clone (or copy) the `custom_dwa_planner` package
If you are reading this file inside the package you can skip this step; otherwise:

```bash
git clone <your-fork-or-source-of>/custom_dwa_planner.git
```

---

## 3. Build the workspace

```bash
cd ~/final_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

> **Important** – make sure the build completes without errors before proceeding.

---

## 4. Environment setup

```bash
# Tell all TurtleBot3 packages which model to simulate
export TURTLEBOT3_MODEL=burger
```
You may want to append the above line to your `~/.bashrc` so it is always exported.

---

## 5. Launch simulation

```bash
# Start Gazebo with the default TurtleBot3 world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Wait until Gazebo has finished loading the world and the robot appears.

---

## 6. Launch the planner

### 6.1. With Nav2 (AMCL + map server)

```bash
ros2 launch custom_dwa_planner navigation.launch.py
```
This will bring up Nav2 together with the custom DWA local planner.

### 6.2. Without Nav2 (local planner only)

```bash
ros2 launch custom_dwa_planner local_planner.launch.py
```
This variant is useful for quickly testing local obstacle avoidance behaviour.

---

## 7. Usage Instructions

### 7.1. Build & run helper snippet
```bash
# Re-build only this package (if you changed its code)
cd ~/final_ws
colcon build --packages-select custom_dwa_planner
source install/setup.bash

# Launch the planner (Nav2)
ros2 launch custom_dwa_planner navigation.launch.py

# Or run the node directly
ros2 run custom_dwa_planner dwa_planner_node
```

### 7.2. Set goals

```bash
#Publisher using goal_publisher.py
ros2 run custom_dwa_planner goal_publisher.py 2 1 0
```

```bash
# Publish a goal from the command line
ros2 topic pub /move_base_simple/goal geometry_msgs/msg/PoseStamped '{header:{frame_id:"map"}, pose:{position:{x:2.0, y:1.0, z:0.0}}}'
```

### 7.3. Monitor debug logs
```bash
# Enable detailed planner logging
ros2 param set /dwa_planner_node ENABLE_DETAILED_LOGGING true

# Tail log files
tail -f ~/final_ws/custom_dwa_planner_debug_*.log

# Monitor console output
ros2 topic echo /rosout
```

The planner automatically writes detailed timing & cost diagnostics to `~/Your_Workspace/custom_dwa_planner_debug_<timestamp>.log`.

### 7.4. Tune parameters on-the-fly
```bash
ros2 param set /dwa_planner_node to_goal_cost_gain 2.0
ros2 param set /dwa_planner_node obs_cost_gain 3.0
ros2 param set /dwa_planner_node max_velocity 0.8

# List all parameters
ros2 param list /dwa_planner_node
```

---

## 8. Troubleshooting

### 8.1. Robot not moving
```bash
# Check sensor data rates
ros2 topic hz /scan /odom /local_map

# Verify the goal was received
ros2 topic echo /move_base_simple/goal

# Check footprint is available
ros2 topic echo /footprint
```

### 8.2. Oscillating behaviour
  • Reduce `obs_cost_gain`  
  • Increase `predict_time_`  
  • Lower `hz_` for more stable planning

### 8.3. Slow / sluggish movement
  • Increase `max_velocity`  
  • Reduce `velocity_samples`, `yawrate_samples`  
  • Decrease `obs_range_` if too conservative

### 8.4. Frequent "No Available Trajectory" errors
  • Check obstacle detection range  
  • Verify footprint size  
  • Increase `velocity_samples` for more options  
  • Review the log file for obstacle density

### 8.5. Helpful debug commands
```bash
# Dump current parameters
ros2 param dump /dwa_planner_node

# Visualise trajectories in RViz
rviz2 -d custom_dwa_planner/rviz/navigation.rviz

# Inspect TF tree
ros2 run tf2_tools view_frames.py
```

---

**Happy navigating!**
