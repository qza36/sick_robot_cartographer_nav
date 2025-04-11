
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='rmslaunch').find('rmslaunch')
    urdf_name = "robot.urdf"
    
    #=====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config'))
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='config.carto.lua')

    
    robot_description = os.path.join(pkg_share, f'urdf/{urdf_name}')

    #下面两行注释掉，by wjb
    # rviz_config_dir = os.path.join(pkg_share)+"/cartographer.rviz"
    # print(f"rviz config in {rviz_config_dir}")

    
    #=====================声明三个节点，cartographer/occupancy_grid_node/rviz_node=================================
    robot_status_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[robot_description]
        )
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id','world',
            '--child-frame-id','odom'
        ]
    )
    pointcloud_to_laserscan_node = Node (
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud_all_fields_fullframe'),
                        ('scan', '/scan')],
            parameters=[{
                #这里的参数 是不是得和雷达或者cartographer的参数一致？ yes
                'use_sim_time': use_sim_time,
                'target_frame': 'laser_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_dir],
        # parameters=[{'use_sim_time': use_sim_time}],
        # output='screen'
        )

    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(robot_status_pub)
    ld.add_action(pointcloud_to_laserscan_node)
    #ld.add_action(tf_static_node)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
