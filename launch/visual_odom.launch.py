from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('jo_navigation'),
        'config',
        'visual_odom.yaml'
    )

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='visodom',
        output='log',
        parameters=[config],
        remappings=[
            ('rgb/image',       '/front_camera/camera/color/image_raw'),
            ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
            ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='visodom',
        output='screen',
        parameters=[config],
        remappings=[
            ('rgb/image',       '/front_camera/camera/color/image_raw'),
            ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
            ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='visodom',
        output='screen',
        parameters=[config],
        remappings=[
            ('rgb/image',       '/front_camera/camera/color/image_raw'),
            ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
            ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    return LaunchDescription([
        rgbd_odometry,
        rtabmap,
        rtabmap_viz,
    ])