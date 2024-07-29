from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    LaunchConfiguration,
    FindExecutable,
    Command,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


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
            FindPackageShare("ugv_descriptions"),
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
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ugv_descriptions"), "launch/view_urdf.rviz"]
    )

    # 发布机器人描述
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # 发布可动关节的相对关系
    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
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
        node_joint_state_publisher,
        node_rviz,
        LogInfo(msg=[rviz_config_file]),
    ]

    return LaunchDescription(declared_arguments + nodes)
