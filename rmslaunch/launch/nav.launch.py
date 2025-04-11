import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_file = os.path.join(get_package_share_directory('rmslaunch'), 'urdf', 'robot.urdf')
    assert os.path.exists(xacro_file), "The robot.urdf doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    zm_robot_cartographer_prefix = get_package_share_directory('rmslaunch')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  zm_robot_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='config.carto.localization.lua')
    
    cartographer_map = os.path.join(
        get_package_share_directory('rmslaunch'),
        'map',
        'cartographer_map.pbstream'
    )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    param_file_name = 'nav2_params.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('rmslaunch'),
            'config',
            param_file_name))


    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id','odom',
            '--child-frame-id','base_link'
            ]
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc,
                 "use_sim_time": use_sim_time}],
            output="screen"),

        Node(
            package="rmslaunch",
            executable="imu_frame_transformer",
            name="imu_frame_transformer",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            ##output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-load_state_filename', cartographer_map]),
        Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),


        Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/cloud_all_fields_fullframe'),
                    ('scan', '/scan')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'cloud_frame',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,  # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
             launch_arguments={'use_sim_time': use_sim_time,
                               'params_file': param_dir}.items(),
         ),

        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2')

    ])