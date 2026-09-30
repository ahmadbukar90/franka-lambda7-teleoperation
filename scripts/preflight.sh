#!/usr/bin/env bash
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ROS 2 is not sourced. Run: source /opt/ros/humble/setup.bash"
  exit 1
fi

missing=0
for package in fd_lambda7_driver_ros2 franka_bringup franka_gripper franka_msgs; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "OK: $package"
  else
    echo "MISSING: $package"
    missing=1
  fi
done

for module in cv2 numpy scipy pydantic torch pyrealsense2; do
  if python3 -c "import $module" >/dev/null 2>&1; then
    echo "OK: Python module $module"
  else
    echo "MISSING: Python module $module"
    missing=1
  fi
done

if [ "$missing" -ne 0 ]; then
  echo "Install the missing items before launching hardware. See README.md."
  exit 1
fi

echo "Preflight passed. Run safe_test.launch.py before connecting the robot."
