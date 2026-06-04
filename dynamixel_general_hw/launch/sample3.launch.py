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
            DeclareLaunchArgument("pan_id", default_value="1"),
            DeclareLaunchArgument("tilt_id", default_value="2"),
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
                    "pan_id": LaunchConfiguration("pan_id"),
                    "tilt_id": LaunchConfiguration("tilt_id"),
                    "robot_description_file": PathJoinSubstitution(
                        [package_share, "urdf", "sample3.urdf"]
                    ),
                    "controllers_file": PathJoinSubstitution(
                        [package_share, "config", "sample3", "default_controllers.yaml"]
                    ),
                    "controllers_to_start": "joint_state_broadcaster position_joint_trajectory_controller",
                }.items(),
            ),
        ]
    )
