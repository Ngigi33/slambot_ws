import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slambot_description"),
            "launch",
            "gazebo.launch.py",
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slambot_controller"),
            "launch",
            "controller.launch.py",
        ),
        # launch_arguments={"use_simple_controller":"False","use"}
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slambot_controller"),
            "launch",
            "joystick_teleop.launch.py",
        )
    )

    return LaunchDescription(
        [
            gazebo,
            controller,
            joystick,
        ]
    )
