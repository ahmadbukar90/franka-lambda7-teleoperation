"""Launch the teleoperation nodes and optional hardware drivers.

Robot motion is disabled unless start_franka:=true is supplied. Review and
calibrate config/safe_defaults.yaml before enabling hardware motion.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("lambda_franka_teleop")
    default_params = os.path.join(package_share, "config", "safe_defaults.yaml")
    franka_share = get_package_share_directory("franka_bringup")
    gripper_share = get_package_share_directory("franka_gripper")

    robot_ip = LaunchConfiguration("robot_ip")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="REPLACE_WITH_ROBOT_IP"),
        DeclareLaunchArgument("namespace", default_value="NS_1"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("start_lambda_driver", default_value="true"),
        DeclareLaunchArgument(
            "fd_sdk_root",
            default_value=[EnvironmentVariable("HOME"), "/sdk-3.17.5"],
        ),
        DeclareLaunchArgument("start_franka", default_value="false"),
        DeclareLaunchArgument("start_gripper", default_value="false"),
        DeclareLaunchArgument("start_camera", default_value="false"),
        DeclareLaunchArgument("start_tactile", default_value="false"),
        DeclareLaunchArgument("start_logger", default_value="false"),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(franka_share, "launch", "example.launch.py")),
            launch_arguments={
                "robot_ips": robot_ip,
                "controller_names": "mobile_cartesian_velocity_example_controller",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_franka")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gripper_share, "launch", "gripper.launch.py")),
            launch_arguments={"robot_ip": robot_ip, "namespace": namespace}.items(),
            condition=IfCondition(LaunchConfiguration("start_gripper")),
        ),
        Node(
            package="lambda_franka_teleop",
            executable="lambda_linear_bridge",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="proximity_adapt_controller",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="lambda_to_franka_gripper",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="d405_tomato_node",
            condition=IfCondition(LaunchConfiguration("start_camera")),
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="d405_operator_view",
            condition=IfCondition(LaunchConfiguration("start_camera")),
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="gelsight_depth_publisher",
            condition=IfCondition(LaunchConfiguration("start_tactile")),
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="gripper_force_relay",
            condition=IfCondition(LaunchConfiguration("start_tactile")),
            output="screen",
        ),
        Node(
            package="lambda_franka_teleop",
            executable="velocity_logger",
            condition=IfCondition(LaunchConfiguration("start_logger")),
            output="screen",
        ),
    ])
