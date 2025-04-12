import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RMS localization system with Nav2."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell in the published occupancy grid'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'
        ),
    ]

    # Get package directories
    rms_package_dir = get_package_share_directory('rmslaunch')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # Define paths to config files
    urdf_path = os.path.join(rms_package_dir, 'urdf', 'robot.urdf')
    assert os.path.exists(urdf_path), f"The robot.urdf doesn't exist in {urdf_path}"

    # Process URDF/XACRO
    robot_description_config = xacro.process_file(urdf_path)
    robot_desc = robot_description_config.toxml()

    # Cartographer configuration
    cartographer_config_dir = os.path.join(rms_package_dir, 'config')
    configuration_basename = 'config.carto.localization.lua'
    cartographer_map = os.path.join(rms_package_dir, 'map', 'cartographer_map.pbstream')

    # Nav2 configuration
    nav2_param_file = os.path.join(rms_package_dir, 'config', 'nav2_params.yaml')

    # Declare Cartographer specific arguments
    cartographer_args = [
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'
        ),
    ]

    # Define nodes
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_publisher',
        output='screen',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link']
    )

    imu_frame_transformer_node = Node(
        package='rmslaunch',
        executable='imu_frame_transformer',
        name='imu_frame_transformer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', cartographer_map
        ]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec
        ]
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/cloud_all_fields_fullframe'),
            ('scan', '/scan')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'cloud_frame',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    # Navigation launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_file
        }.items()
    )

    # Visualization node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Group nodes by functionality for better organization
    sensor_processing_group = GroupAction([
        imu_frame_transformer_node,
        pointcloud_to_laserscan_node
    ])

    localization_group = GroupAction([
        cartographer_node,
        cartographer_occupancy_grid_node
    ])

    # Create the launch description
    ld = LaunchDescription()

    # Add all declared arguments
    for arg in declared_arguments:
        ld.add_action(arg)

    for arg in cartographer_args:
        ld.add_action(arg)

    # Add all nodes and included launch files
    ld.add_action(tf_static_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(sensor_processing_group)
    ld.add_action(localization_group)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)

    return ld