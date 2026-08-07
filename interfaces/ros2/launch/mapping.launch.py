"""Runs the odometry with Scan Context loop closure and pose-graph optimization.

The mapping half of the pair with localization.launch.py. Both drive the same
`process_topics` executable -- the mode is the launch file, not a switch inside
the pipeline -- and each one asserts its own mode and turns the other off, so
whatever the params file says, mapping maps and localization localizes.

What this adds over process_topics.launch.py is `loop_closure.enable`: keyframes
go to Scan Context, accepted loops go into a GTSAM pose graph, and the corrected
result is published on bievr_lio/pgo/{path,odom,map}.

Write the result out with:

  ros2 service call /bievr_lio_mapping_node/save_map_bundle std_srvs/srv/Trigger

which writes cloud.pcd, poses_tum.txt, scan_context.bin and meta.yaml into
`bundle`. That directory is exactly what localization.launch.py takes as `map:=`.

  ros2 launch bievr_lio_ros2 mapping.launch.py \\
      sensor_config:=vbr bundle:=/path/to/bievr_map_bundle
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
    bundle = LaunchConfiguration('bundle').perform(context)

    rviz_config = os.path.join(
        get_package_share_directory('bievr_lio_ros2'), 'rviz', 'mapping.rviz')

    # The node parses plain YAML itself, so the mode is turned on by layering a
    # generated overlay on top of the shared params rather than by a ROS
    # parameter. --params_file is repeatable and later files win per leaf.
    overrides = ['loop_closure:', '  enable: True']
    if bundle:
        overrides.append('  bundle_path: "{}"'.format(bundle))
    overrides += ['localization:', '  enable: False']
    override_path = os.path.join(tempfile.gettempdir(), 'bievr_mapping_overlay.yaml')
    with open(override_path, 'w') as handle:
        handle.write('\n'.join(overrides) + '\n')

    return [
        Node(
            package='bievr_lio_ros2',
            executable='process_topics',
            name='bievr_lio_mapping_node',
            output='screen',
            # Later files win per leaf, so the override lands after the sensor
            # config and turns loop closure on whatever the params file said.
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
            'bundle', default_value='',
            description='Where save_map_bundle writes the map bundle. Empty keeps '
                        'loop_closure.bundle_path from the params file.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 with the mapping visualization config.'),
        OpaqueFunction(function=launch_setup),
    ])
