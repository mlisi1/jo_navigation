import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from glim_common import declare_glim_args, glim_node, visodom_actions  # noqa: E402


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('jo_navigation')


    # LAUNCH PARAMS
    launch_glim_arg = DeclareLaunchArgument(
        'glim',
        default_value='true',
        description='Whether to launch the GLIM stack'
    )


    launch_vslam_arg = DeclareLaunchArgument(
        'vslam',
        default_value='false',
        description='Whether to launch RTAB-Map visual SLAM (mapping mode)'
    )


    launch_glim_view_arg = DeclareLaunchArgument(
        'glim_view',
        default_value='false',
        description='Whether to open the GLIM viewer window (needs a display; GLIM dies if it is lost)'
    )


    launch_visodom_arg = DeclareLaunchArgument(
        'visodom',
        default_value='true',
        description='Whether to launch the visual odometry nodes'
    )


    declare_params_file_cmd = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(pkg_dir, 'config', 'localization_gps.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')



    # LAUNCH FILES
    visodom_launch = os.path.join(pkg_dir, 'launch', 'visual_odom.launch.py')


    # CONFIG FILES
    # INCLUDED LAUNCH FILES
    visodom_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visodom_launch),
        launch_arguments={'vslam': LaunchConfiguration('vslam'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )
    visodom = OpaqueFunction(function=visodom_actions, args=[visodom_include])

    # NODES
    glim = OpaqueFunction(
        function=glim_node,
        args=['gps'],
        condition=IfCondition(LaunchConfiguration('glim')),
    )

    robot_localization_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        remappings=[("odometry/filtered", "odometry/global")],
        parameters=[LaunchConfiguration('localization_params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    def robot_localization_local(context):
        return [Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            remappings=[("odometry/filtered", "odometry/local")],
            parameters=[LaunchConfiguration('localization_params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )]


    # navsat_yaw_offset overrides yaw_offset of the params file (the simulator passes the spawn
    # yaw: there the odometry yaw is 0 when facing east, the real Xsens reads 0 at north)
    navsat_yaw_offset_arg = DeclareLaunchArgument(
        'navsat_yaw_offset', default_value='',
        description='Override navsat_transform yaw_offset [rad] (empty: from the params file)')

    def navsat_transform(context):
        params = [LaunchConfiguration('localization_params'),
                  {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        yaw_offset = LaunchConfiguration('navsat_yaw_offset').perform(context)
        if yaw_offset:
            params.append({'yaw_offset': float(yaw_offset)})
        return [navsat_transform_node(params)]

    navsat_transform_node = lambda params: Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='log',
        parameters=params,
        arguments=['--ros-args','--log-level','WARN'],
        remappings=[
            ('imu', 'imu/data'),
            ('gps/fix', 'gnss'),      # adjust
            ('odometry/filtered', 'odometry/global'), # from local EKF
        ]
    )


    set_zone = TimerAction(
        period=1.0,
        actions=[
            # retried: a single call that fails (e.g. its DDS participant cannot be created at a
            # busy startup) leaves navsat_transform in the wrong UTM zone
            ExecuteProcess(
            cmd=['bash', '-c',
                 'for i in $(seq 1 10); do '
                 'ros2 service call /setUTMZone robot_localization/srv/SetUTMZone '
                 '"{utm_zone: 32N}" && exit 0; sleep 2; done; exit 1'],
            output='screen'
            )
        ]
    )
   


    return LaunchDescription([
        launch_glim_arg,
        launch_glim_view_arg,
        launch_visodom_arg,
        launch_vslam_arg,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        *declare_glim_args(pkg_dir),
        robot_localization_global,
        OpaqueFunction(function=robot_localization_local),
        navsat_yaw_offset_arg,
        OpaqueFunction(function=navsat_transform),
        set_zone,
        glim,
        visodom,
    ])
