# Installation and safety workflow

This package is designed for ROS 2 Humble on Ubuntu 22.04. It packages the
project's own nodes; it does not redistribute the Force Dimension SDK, the
Lambda.7 ROS 2 driver, Franka ROS 2 packages, or vendor device drivers.

## 1. Install external prerequisites

Install and verify the following according to their official documentation:

- ROS 2 Humble
- Force Dimension SDK and `fd_lambda7_driver_ros2`
- Franka ROS 2 stack providing `franka_bringup`, `franka_gripper`, and
  `franka_msgs`
- Intel RealSense SDK / `pyrealsense2`
- GelSight camera access and udev permissions

## 2. Build the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ahmadbukar90/franka-lambda7-teleoperation.git lambda_franka_teleop
cd ..
source /opt/ros/humble/setup.bash
python3 -m pip install -r src/lambda_franka_teleop/requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If `rosdep` cannot find a vendor-specific dependency, install that dependency
using its official instructions, then repeat the build.

## 3. Run the non-actuating test first

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/src/lambda_franka_teleop/scripts/preflight.sh
ros2 launch lambda_franka_teleop safe_test.launch.py
```

The safe test starts the Lambda.7 driver and publishes converted velocity
messages to `/teleop_test/cmd_vel`. It does not start or command a Franka robot.
It assumes the Force Dimension SDK is at `~/sdk-3.17.5`; override this with
`fd_sdk_root:=/path/to/sdk` when needed.

## 4. Enable real hardware only after verification

Review `config/safe_defaults.yaml`, verify the emergency stop, and use a clear
workspace. Replace the robot IP and enable each optional subsystem explicitly:

```bash
ros2 launch lambda_franka_teleop hardware.launch.py \
  robot_ip:=<ROBOT_IP> \
  start_franka:=true \
  start_gripper:=true \
  start_camera:=true \
  start_tactile:=true
```

The supplied limits are conservative starting values, not validated universal
safety limits. Calibrate topic names, frames, camera devices, force gains, and
velocity/acceleration limits for every robot and environment.
