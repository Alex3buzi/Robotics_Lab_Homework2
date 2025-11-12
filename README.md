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

## Launching the Node

Load parameters from `config/kdl_params.yaml` and select controller:

```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=<controller_type>
```

Controller options:

* `velocity_ctrl` – standard velocity
* `velocity_ctrl_null` – null-space velocity
* `vision_ctrl` – vision-based with ArUco

Or run directly for more parameter customization:

```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p ctrl:=<controller_type>
```

---

## Action Server

Execute Cartesian trajectory:

```bash
ros2 action send_goal /execute_trajectory ros2_kdl_package/action/ExecuteTrajectory \
"{traj_duration: 25.0, acc_duration: 10.0, total_time: 25.0, kp: 1.0, end_position_x: 0.7, end_position_y: 0.0, end_position_z: 1.0}"
```

Feedback is published during execution.

---

## Gazebo Service

Update ArUco marker pose:

```bash
ros2 service call /world/aruco_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'arucotag'}, pose: {position: {x: -0.2, y: -0.73, z: 0.48}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

This service can also be accessed through `rqt_service_caller`.
