from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition
from launch.actions import GroupAction



def generate_launch_description():
    
    wheel_radius_arg=DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.0425"
    )
    
    wheel_separation_arg=DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.245"
    )
    
    use_diffdrive_controller_arg=DeclareLaunchArgument(
        "use_diffdrive_controller",
        default_value="True",
    )
    
    
    
    wheel_radius=LaunchConfiguration("wheel_radius")
    wheel_separation=LaunchConfiguration("wheel_separation")
    use_diffdrive_controller=LaunchConfiguration("use_diffdrive_controller")
    
    
    joint_state_broadcaster_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )


    wheel_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "slambot_diffdrive_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=IfCondition(use_diffdrive_controller)
    )
    
    odom_republisher=Node(
        package="slambot_controller",
        executable="odom_republisher"
    )
    
    
    simple_controller= GroupAction(
        condition=UnlessCondition(use_diffdrive_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                     "simple_velocity_controller",
                      "--controller-manager",
                      "/controller_manager" ],
    
                ),
            Node(
                package="slambot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius":wheel_radius,
                            "wheel_separation":wheel_separation}]
                )      
        ]
    )
    
    
    
    
    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_diffdrive_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
        odom_republisher
        
    ])