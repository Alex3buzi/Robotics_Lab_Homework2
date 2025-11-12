
---

# RL25 HW2

This package implements different control modes for the `ros2_kdl_node`, including standard velocity control, null-space velocity control, and a vision-based controller using ArUco markers in Gazebo.
It also includes an action server for executing trajectories and a Gazebo–ROS2 service bridging.

---


## Simulation Environments

### Launch RViz Simulator

```bash
ros2 launch iiwa_description aruco_gazebo.launch.py robot_controller:=velocity_controller command_interface:=velocity
```

### Launch Gazebo Simulator

Launch Gazebo with the robot commanded by a velocity controller.
The world by default is `aruco.world`, which contains an ArUco tag visible to the camera.

```bash
ros2 launch iiwa_description aruco_gazebo.launch.py start_rviz:=false robot_controller:=velocity_controller command_interface:=velocity use_sim:=true
```

**Note:** Launch the simulators in a separate terminal.

---

## Launching the Node

The node loads its parameters automatically from the `config/kdl_params.yaml` file when launched.

You can select the desired controller type with the `ctrl` argument:

```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=velocity_ctrl
```

Possible values for `ctrl` are:

* `velocity_ctrl`
* `velocity_ctrl_null`
* `vision_ctrl`

---

## Controllers

### Null-space Velocity Controller

Run the KDL node with the null-space velocity controller:

```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p ctrl:=velocity_ctrl_null
```

Alternatively, use the launch file:

```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=velocity_ctrl_null
```

---

### Standard Velocity Controller

Run the KDL node with the standard velocity controller:

```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p ctrl:=velocity_ctrl
```

Alternatively, use the launch file:

```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=velocity_ctrl
```

---

## Vision-Based Control

To activate the vision-based controller, set the ROS parameter `ctrl:=vision_ctrl` when launching the node:

```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p ctrl:=vision_ctrl
```
Alternatively, use the launch file:

```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=vision_ctrl
```

Move the ArUco marker in Gazebo to test the “look-at” behavior.

---

## Action Server Command

The `ros2_kdl_node` also acts as an action server that executes a Cartesian linear trajectory defined by timing and target position parameters.
The client sends a goal specifying the desired trajectory duration, acceleration duration, total motion time, proportional gain, and the final end-effector position.

To send a goal to the trajectory execution action server, use:

```bash
ros2 action send_goal /execute_trajectory ros2_kdl_package/action/ExecuteTrajectory \
"{traj_duration: 25.0, acc_duration: 10.0, total_time: 25.0, kp: 1.0, end_position_x: 0.7, end_position_y: 0.0, end_position_z: 1.0}"
```

During execution, the node publishes position error feedback so that the client can monitor tracking performance.


---

## Gazebo Service to Update ArUco Marker Pose

While the Gazebo simulation is running, it is possible to update the ArUco marker pose using the following service call:

```bash
ros2 service call /world/aruco_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'arucotag'}, pose: {position: {x: -0.2, y: -0.73, z: 0.48}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

This service can also be accessed through `rqt_service_caller`.

---

