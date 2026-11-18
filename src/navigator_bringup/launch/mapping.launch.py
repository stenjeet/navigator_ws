import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# pose_source -> (pose topic, map topic, node name)
SOURCES = {
    'ground_truth': ('/ground_truth/odom', '/map/ground_truth', 'occupancy_grid_gt'),
    'estimate': ('/estimate/odom', '/map/estimate', 'occupancy_grid_ekf'),
}


def generate_launch_description():
    """Simulation plus occupancy-grid mapping (stage 6).

    ``pose_source`` selects which poses build the map:

    * ``estimate`` (default) - the EKF estimate from ``/estimate/odom``, so the
      stage-5 localization node is started too;
    * ``ground_truth`` - ``/ground_truth/odom``, the reference map;
    * ``both`` - two mapper instances on the same scans, which is exactly what
      the stage-6 map comparison needs (see ``tools/map_compare.py``).

    ``sensors_params`` / ``ekf_params`` are forwarded to ``simulation.launch.py``
    so the stage-9 scenario runner can supply alternative parameter files.
    """
    bringup_share = get_package_share_directory('navigator_bringup')
    slam_share = get_package_share_directory('navigator_slam')
    sim_share = get_package_share_directory('navigator_sim')
    loc_share = get_package_share_directory('navigator_localization')

    simulation = os.path.join(bringup_share, 'launch', 'simulation.launch.py')
    occupancy_yaml = os.path.join(slam_share, 'config', 'occupancy.yaml')
    sensors_yaml = os.path.join(sim_share, 'config', 'sensors.yaml')
    ekf_yaml = os.path.join(loc_share, 'config', 'ekf.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    pose_source = LaunchConfiguration('pose_source')
    sensors_params = LaunchConfiguration('sensors_params')
    ekf_params = LaunchConfiguration('ekf_params')

    def make_nodes(context, *args, **kwargs):
        source = pose_source.perform(context).strip()
        if source not in ('ground_truth', 'estimate', 'both'):
            raise RuntimeError(
                "pose_source must be one of 'ground_truth', 'estimate', 'both', "
                "got '%s'" % source
            )

        selected = ['ground_truth', 'estimate'] if source == 'both' else [source]
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation),
                launch_arguments={
                    'use_rviz': use_rviz,
                    'use_localization': (
                        'true' if 'estimate' in selected else 'false'
                    ),
                    'sensors_params': sensors_params,
                    'ekf_params': ekf_params,
                }.items(),
            ),
        ]
        for name in selected:
            pose_topic, map_topic, node_name = SOURCES[name]
            actions.append(Node(
                package='navigator_slam',
                executable='occupancy_grid_node',
                name=node_name,
                parameters=[
                    occupancy_yaml,
                    {'pose_topic': pose_topic, 'map_topic': map_topic},
                ],
                output='screen',
            ))
        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 (set false for headless runs).'),
        DeclareLaunchArgument(
            'pose_source', default_value='estimate',
            description="Pose used to build the map: 'estimate', "
                        "'ground_truth' or 'both'."),
        DeclareLaunchArgument(
            'sensors_params', default_value=sensors_yaml,
            description='Parameter file for the UWB and lidar models '
                        '(default: the navigator_sim config).'),
        DeclareLaunchArgument(
            'ekf_params', default_value=ekf_yaml,
            description='Parameter file for the EKF localizer '
                        '(default: the navigator_localization config).'),
        OpaqueFunction(function=make_nodes),
    ])
