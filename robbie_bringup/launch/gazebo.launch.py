# Launch file to start the gazebo and spawn the robot in gazebo model environment

import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robbie_description = get_package_share_directory("robbie_description")
    robbie_description_prefix = get_package_prefix("robbie_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Set the path to the world file
    world_file_name = "turtlebot3_house.world"
    world_path = os.path.join(robbie_description, "worlds", world_file_name)

    rviz_config = os.path.join(get_package_share_path('robbie_bringup'), 'rviz', 'config.rviz')

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        robbie_description, "urdf", "robbie.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    declare_world_cmd = DeclareLaunchArgument(
    name="world",
    default_value=world_path,
    description="Full path to the world model file to load")

    # Define x and y positions for robot spawn
    spawn_x = LaunchConfiguration('spawn_x', default='-1.0')
    spawn_y = LaunchConfiguration('spawn_y', default='-1.0')
    world = LaunchConfiguration('world')
    
    spawn_x_arg = DeclareLaunchArgument(
            'spawn_x',
            default_value='-1.0',
            description='X position for robot spawn')

    spawn_y_arg = DeclareLaunchArgument(
            'spawn_y',
            default_value='-1.0',
            description='Y position for robot spawn')

    model_path = os.path.join(robbie_description, "models")
    model_path += pathsep + os.path.join(robbie_description_prefix, "share")

    # Environment variables are dynamic values that you can access inside the environment your process runs in.
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)  #to properly load gazebo and start urdf model in gazebo

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")),
        launch_arguments={'world': world}.items()
   
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    rviz_node = Node(
		package = 'rviz2',
		executable= 'rviz2',
		arguments= ['-d', rviz_config]
	)

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "robbie",
                                   "-topic", "robot_description",
                                   "-x", spawn_x, "-y", spawn_y],
                        output="screen"
    )

    return LaunchDescription([
        env_var,
        model_arg,
        declare_world_cmd,
        spawn_x_arg,
        spawn_y_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        rviz_node
    ])