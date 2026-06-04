from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("dynamixel_general_hw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="57600"),
            DeclareLaunchArgument("protocol_1_0", default_value="false"),
            DeclareLaunchArgument("dynamixel_id", default_value="0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, "launch", "dynamixel_general_control.launch.py"]
                    )
                ),
                launch_arguments={
                    "port_name": LaunchConfiguration("port_name"),
                    "baud_rate": LaunchConfiguration("baud_rate"),
                    "protocol_1_0": LaunchConfiguration("protocol_1_0"),
                    "dynamixel_id": LaunchConfiguration("dynamixel_id"),
                    "robot_description_file": PathJoinSubstitution(
                        [package_share, "urdf", "sample4.urdf"]
                    ),
                    "controllers_file": PathJoinSubstitution(
                        [package_share, "config", "sample4", "default_controllers.yaml"]
                    ),
                    "controllers_to_start": "joint_state_broadcaster joint_group_velocity_controller",
                }.items(),
            ),
        ]
    )
