"""Runs the odometry with localization against a prior map.

This is the same `process_topics` executable as mapping -- the mode is this
launch file plus `localization.enable`, not a switch inside the pipeline. The
prior map may be a bundle directory (relocalizes by itself) or a bare .pcd from
any SLAM (waits for an RViz "2D Pose Estimate" on /initialpose).

  ros2 launch bievr_lio_ros2 localization.launch.py \\
      sensor_config:=vbr map:=/path/to/bievr_map_bundle
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_config(value, subdir):
    """Resolve a config-file launch argument to a full path.

    An absolute path (starting with '/') is used verbatim so configs can live in
    an external folder; otherwise `value` is treated as a name (without .yaml)
    looked up in this package's installed config/<subdir> directory.
    """
    if value.startswith('/'):
        return value
    pkg_share = get_package_share_directory('bievr_lio_ros2')
    return os.path.join(pkg_share, 'config', subdir, value + '.yaml')


def launch_setup(context, *args, **kwargs):
    sensor_config = LaunchConfiguration('sensor_config').perform(context)
    params = LaunchConfiguration('params').perform(context)
    prior_map = LaunchConfiguration('map').perform(context)

    rviz_config = os.path.join(
        get_package_share_directory('bievr_lio_ros2'), 'rviz', 'localization.rviz')

    # The node parses plain YAML itself, so the mode is turned on by layering a
    # generated overlay on top of the shared params rather than by a ROS
    # parameter. --params_file is repeatable and later files win per leaf.
    overrides = ['localization:', '  enable: True']
    if prior_map:
        overrides.append('  map_path: "{}"'.format(prior_map))
    override_path = os.path.join(tempfile.gettempdir(), 'bievr_localization_overlay.yaml')
    with open(override_path, 'w') as handle:
        handle.write('\n'.join(overrides) + '\n')

    return [
        Node(
            package='bievr_lio_ros2',
            executable='process_topics',
            name='bievr_lio_localization_node',
            output='screen',
            # Later files win per leaf, so the override lands after the sensor
            # config and turns localization on whatever the params file said.
            arguments=[
                '--sensor_config_file', resolve_config(sensor_config, 'sensor_configs'),
                '--params_file', resolve_config(params, ''),
                '--params_file', override_path,
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_config',
            description="Sensor config: a name (without .yaml) in config/sensor_configs/, "
                        "or an absolute path (starting with '/') to a config file."),
        DeclareLaunchArgument(
            'params', default_value='params',
            description="Algorithm params: a name (without .yaml) in config/, "
                        "or an absolute path (starting with '/') to a config file."),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Prior map: a bundle directory or a .pcd file. Empty keeps '
                        'localization.map_path from the params file.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 with the localization visualization config.'),
        OpaqueFunction(function=launch_setup),
    ])
