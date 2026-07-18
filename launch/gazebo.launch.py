import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('mir_200_og_2')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'mir_200_og_2.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'custom_factory.sdf') 

    share_parent = os.path.dirname(pkg_share)
    resource_path = f"{share_parent}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(), 
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mir_200', '-z', '0.3'],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen'
    )

    # NEW FIX: Force the map to connect to odometry immediately
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        rsp_node,
        spawn_node,
        bridge_node,
        static_tf_node
    ])
