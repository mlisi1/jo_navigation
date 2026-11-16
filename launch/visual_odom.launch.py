from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('jo_navigation'),
        'config',
        'visual_odom.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='visodom',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        # Hide the per-frame "Odom: quality=..." INFO spam; warnings still show
        arguments=['--ros-args', '--log-level', 'warn'],
        remappings=[
            ('rgb/image',       '/front_camera/camera/color/image_raw'),
            ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
            ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    # One database per mapping run, next to the GLIM maps in /saved_maps
    def launch_rtabmap(context):
        map_dir = os.path.join('/saved_maps', datetime.now().strftime('%Y%m%d_%H%M%S') + '_rtabmap')
        os.makedirs(map_dir, exist_ok=True)
        return [Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='visodom',
            output='screen',
            parameters=[config, {'database_path': os.path.join(map_dir, 'rtabmap.db'), 'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'warn'],
            remappings=[
                ('rgb/image',       '/front_camera/camera/color/image_raw'),
                ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
                ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
                ('scan_cloud',      '/velodyne_points'),
            ],
        )]

    rtabmap = OpaqueFunction(
        function=launch_rtabmap,
        condition=IfCondition(LaunchConfiguration('vslam')),
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='visodom',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image',       '/front_camera/camera/color/image_raw'),
            ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
            ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'vslam',
            default_value='false',
            description='Whether to launch the RTAB-Map SLAM node (mapping mode, no map -> odom TF)'
        ),
        rgbd_odometry,
        rtabmap,
        # rtabmap_viz,
    ])