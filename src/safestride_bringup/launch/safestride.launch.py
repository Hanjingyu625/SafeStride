"""Launch the SafeStride robot description, serial bridge, and supervisor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [
            FindPackageShare('safestride_bringup'),
            'config',
            'safestride.yaml',
        ]
    )

    config_file = LaunchConfiguration('config_file')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_crosswalk = LaunchConfiguration('enable_crosswalk')
    enable_terrain = LaunchConfiguration('enable_terrain')
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                PathJoinSubstitution(
                    [
                        FindPackageShare('safestride_description'),
                        'urdf',
                        'safestride.urdf.xacro',
                    ]
                ),
                ' wheel_radius:=',
                wheel_radius,
                ' wheel_separation:=',
                wheel_separation,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to the SafeStride ROS parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'wheel_radius',
                default_value='0.15',
                description='Measured powered-wheel radius in metres.',
            ),
            DeclareLaunchArgument(
                'wheel_separation',
                default_value='0.55',
                description='Measured lateral separation of powered wheels.',
            ),
            DeclareLaunchArgument(
                'enable_terrain',
                default_value='true',
                description='Start the Terrain Uno serial sensor bridge.',
            ),
            DeclareLaunchArgument(
                'enable_gps',
                default_value='false',
                description='Start the BE-220 serial GPS adapter.',
            ),
            DeclareLaunchArgument(
                'enable_crosswalk',
                default_value='false',
                description='Start GPS/V2X crosswalk assistance.',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='safestride_bridge',
                executable='serial_bridge_node',
                name='serial_bridge',
                output='screen',
                parameters=[
                    config_file,
                    {
                        'base.wheel_radius_m': ParameterValue(
                            wheel_radius,
                            value_type=float,
                        ),
                        'base.wheel_separation_m': ParameterValue(
                            wheel_separation,
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package='safestride_control',
                executable='safety_supervisor',
                name='safety_supervisor',
                output='screen',
                parameters=[config_file],
            ),
            Node(
                package='safestride_bridge',
                executable='terrain_bridge_node',
                name='terrain_bridge',
                output='screen',
                condition=IfCondition(enable_terrain),
                parameters=[config_file],
            ),
            Node(
                package='safestride_sensors',
                executable='gps_node',
                name='gps_node',
                output='screen',
                condition=IfCondition(enable_gps),
                parameters=[config_file],
            ),
            Node(
                package='safestride_navigation',
                executable='crosswalk_controller',
                name='crosswalk_controller',
                output='screen',
                condition=IfCondition(enable_crosswalk),
                parameters=[config_file],
            ),
        ]
    )
