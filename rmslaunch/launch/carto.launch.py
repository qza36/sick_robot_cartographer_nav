import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RMS mapping system."""

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
            description='Map resolution in meters'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='0.5',
            description='Period for publishing the map in seconds'
        )
    ]

    # Get launch configurations
    pkg_share = FindPackageShare(package='rmslaunch').find('rmslaunch')
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # Define paths to config files
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    config_dir = os.path.join(pkg_share, 'config')
    config_basename = 'config.carto.lua'

    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_path]
    )

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
            'target_frame': 'laser_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    # SLAM related nodes
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename
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

    mapping_group = GroupAction([
        cartographer_node,
        cartographer_occupancy_grid_node
    ])

    # Create the launch description
    ld = LaunchDescription()

    # Add all declared arguments
    for arg in declared_arguments:
        ld.add_action(arg)

    # Add all nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(tf_static_node)
    ld.add_action(sensor_processing_group)
    ld.add_action(mapping_group)
    ld.add_action(rviz_node)

    return ld