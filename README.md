# Lambda.7–Franka FR3 ROS 2 Teleoperation

A ROS 2 Humble velocity–velocity teleoperation framework for a Force Dimension Lambda.7 haptic device and Franka Research 3 robot. It includes Cartesian velocity control, adaptive proximity force feedback, Franka gripper control, Intel RealSense D405 operator vision, and GelSight tactile contact feedback.

## Safety

This software commands a real robot. Test first with the robot in a safe, clear workspace, use reduced velocity limits, and keep the emergency stop available. Check all topic names, coordinate frames, force limits, and the robot IP before operation.

## Dependencies

Install these separately; their source code is not included in this repository:

- Ubuntu 22.04 and ROS 2 Humble
- Force Dimension SDK and the `fd_lambda7_driver_ros2` ROS 2 driver
- Franka ROS 2 packages for FR3, including `franka_bringup`, `franka_gripper`, and `franka_msgs`
- Intel RealSense SDK / Python package `pyrealsense2`
- ROS 2 `cv_bridge`, OpenCV, NumPy, and PyTorch
- GelSight Mini hardware and camera access

## Start the Lambda.7 driver

Open a terminal:

```bash
source /opt/ros/humble/setup.bash
export FD_SDK_ROOT="$HOME/sdk-3.17.5"
export LD_LIBRARY_PATH="$FD_SDK_ROOT/lib/release/lin-x86_64-gcc:$LD_LIBRARY_PATH"

ros2 run fd_lambda7_driver_ros2 driver_node --ros-args \
  -p device_id:=-1 \
  -p rate_hz:=500.0
```

## Start the Franka FR3

In another terminal, replace `<ROBOT_IP>` with your robot address:

```bash
source /opt/ros/humble/setup.bash

ros2 launch franka_bringup example.launch.py \
  robot_ips:=<ROBOT_IP> \
  controller_names:="mobile_cartesian_velocity_example_controller"
```

Start the gripper:

```bash
ros2 launch franka_gripper gripper.launch.py \
  robot_ip:=<ROBOT_IP> \
  namespace:=NS_1
```

## Start teleoperation nodes

From this repository folder:

```bash
source /opt/ros/humble/setup.bash
python3 teleop_nodes/lambda_linear_bridge.py --ros-args
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
python3 teleop_nodes/Proximity_Adapt_controller.py
```

In a third terminal:

```bash
source /opt/ros/humble/setup.bash
python3 teleop_nodes/lambda_to_franka_gripper.py --ros-args
```

## Start RealSense D405 proximity feedback

```bash
source /opt/ros/humble/setup.bash
python3 camera/d405_tomato_ros_node.py
```

For the operator view:

```bash
source /opt/ros/humble/setup.bash
python3 camera/d405_operator_view.py
```

Optional ROS topic checks:

```bash
ros2 topic echo /d405/target_depth_cm
ros2 topic echo /d405/tomato_detected
```

## Start GelSight tactile force feedback

Run the publisher from the `tactile` folder so it can find its model file:

```bash
source /opt/ros/humble/setup.bash
cd tactile
python3 Gelsight_depth_publisher.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
python3 tactile/gripper_force_relay.py
```

## Optional velocity/force logging

```bash
source /opt/ros/humble/setup.bash
python3 teleop_nodes/log_lambda_franka_velocities.py
```

Generated CSV logs are intentionally ignored by Git.

## Repository contents

- `teleop_nodes/`: Lambda.7–Franka bridge, adaptive controller, gripper mapping, and logger
- `camera/`: RealSense D405 detection and operator-view nodes
- `tactile/`: GelSight tactile depth publisher, force relay, configuration, utilities, and model

## Potential application domains

This framework provides a modular basis for research and prototyping of bilateral teleoperation with high-fidelity haptic cues. Potential application domains include:

- **Agriculture:** remote crop inspection, delicate fruit harvesting, and contact-aware manipulation in greenhouse or field environments.
- **Healthcare and medical robotics:** research on remote manipulation, training simulators, and contact-rich assistance tasks. Clinical deployment requires separate safety validation, regulatory approval, and human-factors evaluation.
- **Marine and subsea teleoperation:** contact-aware inspection and manipulation in underwater or hazardous environments where direct human access is difficult.

The Lambda.7 haptic device, Cartesian force feedback, RealSense proximity sensing, and GelSight tactile sensing can support richer operator perception during remote manipulation. Performance and safety must be validated separately for each target application.
