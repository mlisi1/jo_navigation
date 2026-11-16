import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from glim_common import declare_glim_args, glim_node, visodom_actions  # noqa: E402


def generate_launch_description():
    pkg_dir = get_package_share_directory('jo_navigation')

    # LAUNCH PARAMS
    launch_glim_arg = DeclareLaunchArgument(
        'glim',
        default_value='true',
        description='Whether to launch the GLIM stack'
    )

    launch_glim_view_arg = DeclareLaunchArgument(
        'glim_view',
        default_value='false',
        description='Whether to open the GLIM viewer window (needs a display)'
    )

    launch_visodom_arg = DeclareLaunchArgument(
        'visodom',
        default_value='true',
        description='Whether to launch the visual odometry nodes (skipped if RTAB-Map is not installed)'
    )

    launch_vslam_arg = DeclareLaunchArgument(
        'vslam',
        default_value='false',
        description='Whether to launch RTAB-Map visual SLAM (mapping mode)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(pkg_dir, 'config', 'localization.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # INCLUDED LAUNCH FILES
    visodom_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'visual_odom.launch.py')),
        launch_arguments={'vslam': LaunchConfiguration('vslam'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )
    visodom = OpaqueFunction(function=visodom_actions, args=[visodom_include])

    # NODES
    glim = OpaqueFunction(
        function=glim_node,
        args=['local'],
        condition=IfCondition(LaunchConfiguration('glim')),
    )

    def ekf(context):
        return [Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('localization_params'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )]

    return LaunchDescription([
        launch_glim_arg,
        launch_glim_view_arg,
        launch_visodom_arg,
        launch_vslam_arg,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        *declare_glim_args(pkg_dir),
        OpaqueFunction(function=ekf),
        glim,
        visodom,
    ])
