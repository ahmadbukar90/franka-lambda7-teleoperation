from glob import glob
from setuptools import find_packages, setup

package_name = "lambda_franka_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
    ],
    package_data={
        package_name: [
            "tactile/default_config.json",
            "tactile/models/*.pt",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Ahmad Abubakar",
    maintainer_email="ahmadbukar90@gmail.com",
    description="Lambda.7–Franka FR3 teleoperation with optional RealSense and GelSight sensing.",
    license="Proprietary; contact the repository owner for reuse permissions.",
    entry_points={
        "console_scripts": [
            "lambda_linear_bridge = lambda_franka_teleop.lambda_linear_bridge:main",
            "proximity_adapt_controller = lambda_franka_teleop.proximity_adapt_controller:main",
            "lambda_to_franka_gripper = lambda_franka_teleop.lambda_to_franka_gripper:main",
            "velocity_logger = lambda_franka_teleop.log_lambda_franka_velocities:main",
            "d405_tomato_node = lambda_franka_teleop.camera.d405_tomato_ros_node:main",
            "d405_operator_view = lambda_franka_teleop.camera.d405_operator_view:main",
            "gelsight_depth_publisher = lambda_franka_teleop.tactile.gelsight_depth_publisher:main",
            "gripper_force_relay = lambda_franka_teleop.tactile.gripper_force_relay:main",
        ],
    },
)
