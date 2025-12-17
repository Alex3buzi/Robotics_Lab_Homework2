# RL25 HW2

This package implements different control modes for the `ros2_kdl_node` (standard velocity, null-space velocity, vision-based with ArUco markers) and includes an action server for executing trajectories.

---

## Setup

Build the workspace and source the setup file before running anything:

```bash
colcon build
source install/setup.bash
```

> **Note:** Every new terminal must `source install/setup.bash`.

---

## Simulation

### Launch RViz or Gazebo

Open **two separate terminals**: one for the simulator, one for the control node.

**RViz:**

```bash
ros2 launch iiwa_description aruco_gazebo.launch.py robot_controller:=velocity_controller command_interface:=velocity
```

**Gazebo (with ArUco marker):**

```bash
ros2 launch iiwa_description aruco_gazebo.launch.py start_rviz:=false robot_controller:=velocity_controller command_interface:=velocity use_sim:=true
```

---

## Launching the Action Server
Launch the main KDL node. This node acts as an Action Server. It initializes the robot and controllers but waits for a goal before moving.

## To run with Velocity-ctrl (Standard velocity):
```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=velocity_ctrl
```

## To run with Null-Space Velocity Control (Kinematic Control):
```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=velocity_ctrl_null
```

## To run with Vision-Based Control:
```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=vision_ctrl
```

---

## Send Trajectory Goal

Execute Cartesian trajectory:
To execute the trajectory, launch the Action Client. This node loads the parameters from config/kdl_params.yaml and sends the goal to the server.

```bash
ros2 launch ros2_kdl_package kdl_client.launch.py
```
##Configuration

You can modify the trajectory points and duration by editing the YAML file:
- File: src/ros2_kdl_package/config/kdl_params.yaml
- Parameters: traj_duration, end_position_x, end_position_y, end_position_z, etc.

---

## Gazebo Service

Update ArUco marker pose:

```bash
ros2 service call /world/aruco_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'arucotag'}, pose: {position: {x: -0.2, y: -0.73, z: 0.48}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

This service can also be accessed through `rqt_service_caller`.
