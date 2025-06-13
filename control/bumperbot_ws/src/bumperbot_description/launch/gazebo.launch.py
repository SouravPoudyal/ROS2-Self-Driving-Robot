import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    world_path = PathJoinSubstitution([
        bumperbot_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    model_path = str(Path(bumperbot_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(bumperbot_description_dir, "models")

    robot_description = ParameterValue(Command([
        "xacro ", 
        LaunchConfiguration("model"),
        " is_ignition:=",
        is_ignition
        ]),
        value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
        )
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={
        "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "bumperbot"
        ],
        output="screen"
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msg/msg/Imu[gz.msgs.IMU"],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])