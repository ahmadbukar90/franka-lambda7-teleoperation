"""Launch a non-actuating Lambda.7 input test.

This launch file never starts the Franka robot and routes velocity output to a
diagnostic topic. It is intended for verifying the Lambda.7 driver, button
gate, and velocity conversion before any robot is connected.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("lambda_franka_teleop")
    default_params = os.path.join(package_share, "config", "safe_defaults.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("start_lambda_driver", default_value="true"),
        DeclareLaunchArgument(
            "fd_sdk_root",
            default_value=[EnvironmentVariable("HOME"), "/sdk-3.17.5"],
        ),
        DeclareLaunchArgument("device_id", default_value="-1"),
        DeclareLaunchArgument("rate_hz", default_value="500.0"),
        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            [
                LaunchConfiguration("fd_sdk_root"),
                "/lib/release/lin-x86_64-gcc:",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ],
        ),
        Node(
            package="fd_lambda7_driver_ros2",
            executable="driver_node",
            name="lambda7_driver",
            parameters=[{
                "device_id": ParameterValue(LaunchConfiguration("device_id"), value_type=int),
                "rate_hz": ParameterValue(LaunchConfiguration("rate_hz"), value_type=float),
            }],
            condition=IfCondition(LaunchConfiguration("start_lambda_driver")),
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="lambda_linear_bridge",
            name="lambda_linear_bridge",
            parameters=[
                LaunchConfiguration("params_file"),
                {"output_topic": "/teleop_test/cmd_vel"},
            ],
            output="screen",
        ),
    ])
