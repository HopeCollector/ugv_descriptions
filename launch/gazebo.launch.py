from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import (
    LaunchConfiguration,
    FindExecutable,
    Command,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from math import pi

PKG_NAME = "ugv_descriptions"


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "model_name",
            default_value="",
            description="Name of the model to view.",
        )
    )

    model_base_dir = PathJoinSubstitution(
        [
            FindPackageShare(PKG_NAME),
            "models",
            LaunchConfiguration("model_name"),
        ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    model_base_dir,
                    "urdf/all.xacro",
                ]
            ),
            " use_gazebo:=True",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(PKG_NAME), "launch/gazebo.rviz"]
    )

    gz_bdg_config_file = PathJoinSubstitution([model_base_dir, "gz_bdg.yml"])

    # 发布机器人描述
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # gazebo
    node_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items(),
    )

    node_gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            LaunchConfiguration("model_name"),
            "-z",
            "0.5",
        ],
    )

    node_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": gz_bdg_config_file,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    controller_name = "mecanum_controller_x3_mecanum"
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
        ],
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        node_robot_state_publisher,
        node_gazebo,
        node_gz_spawn_entity,
        node_gz_bridge,
        # node_rviz,
        robot_controller_spawner,
        LogInfo(msg=[rviz_config_file]),
    ]

    return LaunchDescription(declared_arguments + nodes)
