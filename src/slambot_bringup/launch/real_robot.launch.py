import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slambot_firmware"),
            "launch",
            "hardware_interface.launch.py",
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slambot_controller"),
            "launch",
            "controller.launch.py",
        ),
        # launch_arguments={"use_diffdrive_controller":"True"}.items()
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
            hardware_interface,
            controller,
            joystick,
        ]
    )
