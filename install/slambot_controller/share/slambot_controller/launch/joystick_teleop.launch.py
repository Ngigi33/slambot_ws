from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    
    # joystick driver; read joystick command and publishh to ros2
    joystick_node=Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("slambot_controller"),"config","joystick_config.yaml")]

    )
    
    # will pubish velocity message for our controller
    joystick_teleop=Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("slambot_controller"),"config","joystick_teleop.yaml")]
    )
    
    return LaunchDescription([
        joystick_node,
        joystick_teleop
        
    ])