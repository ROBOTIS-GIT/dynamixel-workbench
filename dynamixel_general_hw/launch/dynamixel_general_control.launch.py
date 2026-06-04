from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controller_spawners(context):
    controllers = LaunchConfiguration("controllers_to_start").perform(context).split()
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for controller in controllers
    ]


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    protocol_1_0 = LaunchConfiguration("protocol_1_0")
    dynamixel_id = LaunchConfiguration("dynamixel_id")
    pan_id = LaunchConfiguration("pan_id")
    tilt_id = LaunchConfiguration("tilt_id")
    robot_description_file = LaunchConfiguration("robot_description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rvizconfig = LaunchConfiguration("rvizconfig")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " port_name:=",
            port_name,
            " baud_rate:=",
            baud_rate,
            " protocol_1_0:=",
            protocol_1_0,
            " dynamixel_id:=",
            dynamixel_id,
            " pan_id:=",
            pan_id,
            " tilt_id:=",
            tilt_id,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="57600"),
            DeclareLaunchArgument("protocol_1_0", default_value="false"),
            DeclareLaunchArgument("dynamixel_id", default_value="0"),
            DeclareLaunchArgument("pan_id", default_value="1"),
            DeclareLaunchArgument("tilt_id", default_value="2"),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dynamixel_general_hw"), "urdf", "sample1.urdf"]
                ),
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("dynamixel_general_hw"),
                        "config",
                        "sample1_2",
                        "default_controllers.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "controllers_to_start",
                default_value="joint_state_broadcaster position_joint_trajectory_controller",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rvizconfig",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dynamixel_general_hw"), "config", "sample_robot.rviz"]
                ),
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_file],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            OpaqueFunction(function=controller_spawners),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rvizconfig],
                condition=IfCondition(launch_rviz),
                output="screen",
            ),
        ]
    )
