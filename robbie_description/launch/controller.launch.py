#Launch file to launch the functionality of ros2_control including differential kinematics

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.095",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.33282",
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # Spawner of joint state broadcaster to publish /joint_states message that robot_state_publisher uses to publish wheel transform
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Load controller-Spawner of diff_drive_controller
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robbie_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ]
    )

    return LaunchDescription(
        [
            wheel_radius_arg,
            wheel_separation_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,          
        ]
    )