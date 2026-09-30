# Detailed operating and debugging guide

This guide is for researchers who need to understand, configure, test, or
debug the Lambda.7--Franka FR3 teleoperation framework. Start with the
[quick-start guide](../README.md) for installation and the non-actuating test.

> **Safety notice:** This framework can command a physical robot. Test each
> stage at reduced limits, keep the emergency stop available, and validate the
> configuration for the specific robot, tool, environment, and operator. The
> provided values are conservative starting values, not universal safety
> limits.

## 1. System overview

The framework has five optional functional groups:

| Group | Purpose | Main executable(s) |
| --- | --- | --- |
| Lambda.7 input | Reads haptic-device motion and buttons | `fd_lambda7_driver_ros2 driver_node` |
| Cartesian teleoperation | Converts, limits, and sends velocity commands | `lambda_linear_bridge`, `proximity_adapt_controller` |
| Franka gripper | Maps the Lambda.7 gripper angle to a Franka gripper action | `lambda_to_franka_gripper` |
| Vision | Publishes D405 colour/depth data and an operator view | `d405_tomato_node`, `d405_operator_view` |
| Tactile feedback | Estimates GelSight contact/depth and relays grip-force feedback | `gelsight_depth_publisher`, `gripper_force_relay` |

The optional `velocity_logger` records the main motion and force-feedback
signals to a CSV file for experiments.

## 2. ROS data flow

The primary Cartesian-velocity chain is:

| Input topic | Message type | Producing component | Consuming component / output |
| --- | --- | --- | --- |
| `/twist` | `geometry_msgs/TwistStamped` | Lambda.7 driver | `lambda_linear_bridge` |
| `/buttons` | `std_msgs/UInt32` | Lambda.7 driver | Cartesian and gripper bridges |
| `/cmd_vel_raw` | `geometry_msgs/TwistStamped` | `lambda_linear_bridge` | `proximity_adapt_controller` |
| `/cmd_vel_filtered_pre_depth` | `geometry_msgs/TwistStamped` | `proximity_adapt_controller` | `velocity_logger` (diagnostic) |
| `/NS_1/mobile_cartesian_velocity_controller/cmd_vel` | `geometry_msgs/TwistStamped` | `proximity_adapt_controller` | Franka Cartesian velocity controller |
| `/rgb_depth` | `std_msgs/Float64` | D405 node | `proximity_adapt_controller` |
| `/cmd/force_feedback/wrench` | `geometry_msgs/WrenchStamped` | `proximity_adapt_controller` | Lambda.7 driver / haptic-feedback path |

The gripper and tactile chain is:

| Input topic | Message type | Producing component | Consuming component / output |
| --- | --- | --- | --- |
| `/gripper/angle_rad` | `std_msgs/Float64` | Lambda.7 driver | `lambda_to_franka_gripper` |
| `/NS_1/franka_gripper/move` | `franka_msgs/action/Move` | `lambda_to_franka_gripper` | Franka gripper action server |
| `/gelsight/desired_grip_force` | `std_msgs/Float64` | GelSight node | `gripper_force_relay` |
| `/cmd/force_feedback/gripper_force` | `std_msgs/Float64` | `gripper_force_relay` | Lambda.7 driver / haptic-feedback path |

The default namespace is `NS_1`. Change the namespace, controller topic, and
gripper action together if your Franka setup uses a different namespace.

## 3. Recommended staged startup

Use this order. Do **not** begin with full robot motion.

### Stage A -- build and verify dependencies

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lambda_franka_teleop --symlink-install
source install/setup.bash
bash src/lambda_franka_teleop/scripts/preflight.sh
```

Every preflight item must say `OK`. The script checks the expected ROS
packages and Python modules; it cannot prove that hardware is connected or
calibrated.

### Stage B -- non-actuating Lambda.7 test

With the Franka robot disconnected or disabled, run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lambda_franka_teleop safe_test.launch.py
```

This starts the Lambda.7 driver and `lambda_linear_bridge`, but sends output
only to `/teleop_test/cmd_vel`. It cannot start the Franka controller or move
the robot.

If the Force Dimension SDK is not at `~/sdk-3.17.5`, supply its actual path:

```bash
ros2 launch lambda_franka_teleop safe_test.launch.py \
  fd_sdk_root:=/path/to/force-dimension-sdk
```

For a package-only test with no Lambda.7 connected:

```bash
ros2 launch lambda_franka_teleop safe_test.launch.py start_lambda_driver:=false
```

### Stage C -- full integrated launch

After the safe test, emergency-stop check, network verification, and review
of `config/safe_defaults.yaml`, start all required subsystems:

```bash
ros2 launch lambda_franka_teleop hardware.launch.py \
  robot_ip:=<ROBOT_IP> \
  start_franka:=true \
  start_gripper:=true \
  start_camera:=true \
  start_tactile:=true
```

Add `start_logger:=true` when an experiment should be logged. Disable an
unavailable optional sensor explicitly, for example `start_tactile:=false`.

## 4. Manual component startup for diagnosis

Run each command in a separate terminal. In every terminal, first run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 4.1 Lambda.7 driver

```bash
export FD_SDK_ROOT=$HOME/sdk-3.17.5
export LD_LIBRARY_PATH=$FD_SDK_ROOT/lib/release/lin-x86_64-gcc:$LD_LIBRARY_PATH
ros2 run fd_lambda7_driver_ros2 driver_node --ros-args \
  -p device_id:=-1 -p rate_hz:=500.0
```

Expected inputs include `/twist`, `/buttons`, and `/gripper/angle_rad` from
the installed driver. Confirm exact topic names on the target computer with
`ros2 topic list -t`.

### 4.2 Franka arm and gripper

```bash
ros2 launch franka_bringup example.launch.py \
  robot_ips:=<ROBOT_IP> \
  controller_names:="mobile_cartesian_velocity_example_controller"
```

In another terminal:

```bash
ros2 launch franka_gripper gripper.launch.py \
  robot_ip:=<ROBOT_IP> namespace:=NS_1
```

Do not replace `<ROBOT_IP>` with an address copied from another laboratory.
Use the IP address configured for your own robot network.

### 4.3 Project nodes

Use the installed package executables rather than running Python files by
absolute path:

```bash
ros2 run lambda_franka_teleop lambda_linear_bridge --ros-args \
  --params-file ~/ros2_ws/src/lambda_franka_teleop/config/safe_defaults.yaml
```

```bash
ros2 run lambda_franka_teleop proximity_adapt_controller --ros-args \
  --params-file ~/ros2_ws/src/lambda_franka_teleop/config/safe_defaults.yaml
```

```bash
ros2 run lambda_franka_teleop lambda_to_franka_gripper --ros-args \
  --params-file ~/ros2_ws/src/lambda_franka_teleop/config/safe_defaults.yaml
```

### 4.4 D405 vision

```bash
ros2 run lambda_franka_teleop d405_tomato_node
```

```bash
ros2 run lambda_franka_teleop d405_operator_view
```

Useful checks:

```bash
ros2 topic echo /d405/target_depth_cm
ros2 topic echo /d405/tomato_detected
ros2 topic hz /rgb_depth
```

`rqt_image_view` can be used to inspect `/d405/operator_rgb_image`,
`/d405/cv_image`, or `/d405/depth_image` when the ROS image-view plugin is
installed.

### 4.5 GelSight tactile feedback

```bash
ros2 run lambda_franka_teleop gelsight_depth_publisher
```

```bash
ros2 run lambda_franka_teleop gripper_force_relay
```

The GelSight publisher requires access to both configured GelSight cameras and
the included `nnmini.pt` depth model. It publishes contact, depth, grasp state,
and desired grip-force topics. Inspect them with:

```bash
ros2 topic echo /gelsight/contact_state
ros2 topic echo /fruit_grasp
ros2 topic echo /cmd/force_feedback/gripper_force
```

### 4.6 Experiment logger

```bash
ros2 run lambda_franka_teleop velocity_logger
```

The logger subscribes to the Lambda.7 velocity, raw and filtered command
velocities, Franka state, and wrench feedback. Start it only after the relevant
publishers are active.

## 5. Configuration and calibration

Edit `config/safe_defaults.yaml` in your workspace before real-robot tests.
The file contains the package-level parameters for these nodes:

| Node | Important configuration |
| --- | --- |
| `lambda_linear_bridge` | input/output topics, axis signs, motion-enable button, velocity limits, ramps |
| `smoothed_cmd_vel` | Franka controller topic, command timeout, velocity/acceleration limits, depth thresholds, force-feedback limit |
| `lambda_gripper_bridge` | Lambda.7 gripper input, Franka action name, width limits, gripper speed, enable-button requirement |

Start with the supplied low limits. In particular, do not increase velocity,
acceleration, or force-feedback limits until axes, frames, button enablement,
and emergency stop have been verified on your setup.

## 6. Debugging checklist

Run these checks in order when the framework does not start or behaves
unexpectedly:

```bash
ros2 pkg executables lambda_franka_teleop
ros2 node list
ros2 topic list -t
ros2 topic info /twist
ros2 topic info /cmd_vel_raw
ros2 topic info /NS_1/mobile_cartesian_velocity_controller/cmd_vel
```

Then check the signals moving through the chain:

```bash
ros2 topic echo /buttons
ros2 topic echo /twist
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel_filtered_pre_depth
ros2 topic echo /rgb_depth
ros2 topic echo /cmd/force_feedback/wrench
```

For Franka state verification:

```bash
ros2 topic echo --once /NS_1/franka_robot_state_broadcaster/robot_state
```

### Common failures

| Symptom | Likely cause | First action |
| --- | --- | --- |
| `libdhd.so.3: cannot open shared object file` | Force Dimension SDK library path is missing | Use `fd_sdk_root:=...` in the launch command, or export `LD_LIBRARY_PATH` as in Section 4.1. |
| `Package ... not found` | Workspace has not been built or sourced | Run `colcon build --packages-select lambda_franka_teleop --symlink-install`, then `source ~/ros2_ws/install/setup.bash`. |
| No `/twist` or `/buttons` | Lambda.7 driver is not running or cannot see the device | Run the driver alone and inspect `ros2 topic list -t`. |
| `/cmd_vel_raw` changes but Franka does not move | Franka bringup/controller is unavailable, wrong namespace, or output topic mismatch | Check the controller topic and `robot_state`; confirm the controller launch completed without errors. |
| Motion is always zero | Enable button not active, deadband, timeout, or safety scaling is active | Inspect `/buttons`, `/twist`, `/cmd_vel_raw`, and the configured enable-button logic. |
| No D405 depth topic | Camera/RealSense permission or SDK issue | Run `d405_tomato_node` alone and check its terminal output before starting the full launch. |
| GelSight node cannot start | Camera permissions/device paths or model configuration issue | Run it alone, verify both cameras are visible, then check `tactile/default_config.json`. |
| No haptic feedback | Feedback topics or driver-side feedback configuration differ | Inspect `/cmd/force_feedback/wrench` and `/cmd/force_feedback/gripper_force`; compare the driver documentation and active topic names. |

## 7. Reporting an issue

When reporting a reproducible problem, include:

1. Ubuntu and ROS 2 version.
2. Commit ID (`git rev-parse --short HEAD`).
3. The exact command used.
4. Complete terminal error output.
5. Output of `bash scripts/preflight.sh`.
6. Output of `ros2 topic list -t` after startup.
7. Which hardware was physically connected.

Do not publish robot IP addresses, passwords, access tokens, or private
experiment data in a public issue.
