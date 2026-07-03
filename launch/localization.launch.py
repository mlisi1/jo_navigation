import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('jo_navigation')

    # LAUNCH PARAMS
    launch_glim_arg = DeclareLaunchArgument(
        'glim',
        default_value='true',
        description='Whether to launch the GLIM stack'
    )


    launch_visodom_arg = DeclareLaunchArgument(
        'visodom',
        default_value='true',
        description='Whether to launch the visual odometry nodes'
    )


    declare_params_file_cmd = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(pkg_dir, 'config', 'localization.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    


    # LAUNCH FILES
    visodom_launch = os.path.join(pkg_dir, 'launch', 'visual_odom.launch.py')


    # CONFIG FILES
    glim_config = os.path.join(pkg_dir, 'config', 'glim', 'glim_config_bunker')



    # INCLUDED LAUNCH FILES

    visodom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visodom_launch),          
        condition=IfCondition(LaunchConfiguration('visodom'))
    )   




    # NODES
    glim = Node(
        package='glim_ros',
        executable='glim_rosnode',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('glim')),
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '0',
        },
        parameters=[
            {'config_path': glim_config},
            ],
    )
 

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('localization_params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    


    return LaunchDescription([
        launch_glim_arg,
        launch_visodom_arg,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        robot_localization_node,
        glim,
        visodom
    ])
