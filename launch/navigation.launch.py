import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('neobotix_mpo_500')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Point to the map and params you just created
    map_file = os.path.join(pkg_share, 'maps', 'factory_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Include the official Nav2 bringup launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
