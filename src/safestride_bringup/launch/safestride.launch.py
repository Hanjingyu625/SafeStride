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
    enable_foxglove = LaunchConfiguration('enable_foxglove')
    enable_cruise = LaunchConfiguration('enable_cruise')
    enable_terrain = LaunchConfiguration('enable_terrain')
    enable_perception = LaunchConfiguration('enable_perception')
    perception_model_path = LaunchConfiguration('perception_model_path')
    perception_classes_path = LaunchConfiguration('perception_classes_path')
    perception_camera_index = LaunchConfiguration('perception_camera_index')
    perception_camera_backend = LaunchConfiguration(
        'perception_camera_backend'
    )
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
                'enable_perception',
                default_value='false',
                description='Start fail-safe road-surface perception.',
            ),
            DeclareLaunchArgument(
                'perception_model_path',
                default_value='',
                description='Absolute path to the TorchScript surface model.',
            ),
            DeclareLaunchArgument(
                'perception_classes_path',
                default_value='',
                description='Absolute path to the model class JSON file.',
            ),
            DeclareLaunchArgument(
                'perception_camera_index',
                default_value='0',
                description='Linux video-device index used by OpenCV.',
            ),
            DeclareLaunchArgument(
                'perception_camera_backend',
                default_value='v4l2',
                description='OpenCV camera backend: v4l2 or auto.',
            ),
            DeclareLaunchArgument(
                'enable_cruise',
                default_value='true',
                description=(
                    'Publish the default straight-line request; explicit '
                    'motor enable remains required.'
                ),
            ),
            DeclareLaunchArgument(
                'enable_gps',
                default_value='true',
                description=(
                    'Publish BE-220 data received by the Terrain Uno.'
                ),
            ),
            DeclareLaunchArgument(
                'enable_crosswalk',
                default_value='true',
                description='Start safe crosswalk readiness monitor.',
            ),
            DeclareLaunchArgument(
                'enable_foxglove',
                default_value='false',
                description='Expose read-only SafeStride topics on port 8765.',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='safestride_control',
                executable='cruise_command',
                name='cruise_command',
                output='screen',
                condition=IfCondition(enable_cruise),
                parameters=[config_file],
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
                parameters=[
                    config_file,
                    {
                        'require_surface_condition': ParameterValue(
                            enable_perception,
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package='safestride_perception',
                executable='surface_perception',
                name='surface_perception',
                output='screen',
                condition=IfCondition(enable_perception),
                parameters=[
                    config_file,
                    {
                        'model.path': perception_model_path,
                        'model.classes_path': perception_classes_path,
                        'camera.index': ParameterValue(
                            perception_camera_index,
                            value_type=int,
                        ),
                        'camera.backend': perception_camera_backend,
                    },
                ],
            ),
            Node(
                package='safestride_bridge',
                executable='terrain_bridge_node',
                name='terrain_bridge',
                output='screen',
                condition=IfCondition(enable_terrain),
                parameters=[
                    config_file,
                    {
                        'gps.enabled': ParameterValue(
                            enable_gps,
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package='safestride_navigation',
                executable='crosswalk_controller',
                name='crosswalk_controller',
                output='screen',
                condition=IfCondition(enable_crosswalk),
                parameters=[config_file],
            ),
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                condition=IfCondition(enable_foxglove),
                parameters=[config_file],
            ),
        ]
    )
