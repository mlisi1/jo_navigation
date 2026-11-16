"""Helpers shared by localization.launch.py and localization_gps.launch.py.

GLIM runs with the mlisi1/glim_ros2@dev ROS interface: TF ownership, odometry frame and map saving
are set through ROS parameters; the EKF configs fuse ``glim_ros/odometry/local`` (optimized,
expressed in base_link).
"""
import os
import re

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VIEWER = 'libstandard_viewer.so'


def declare_glim_args(pkg_dir):
    return [
        DeclareLaunchArgument(
            'glim_config',
            default_value=os.path.join(pkg_dir, 'config', 'glim', 'glim_config_bunker'),
            description='GLIM config directory (the simulator uses glim_config_bunker_sim)'),
    ]


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def _extension_modules(config_dir):
    with open(os.path.join(config_dir, 'config_ros.json')) as f:
        text = re.sub(r'//[^\n]*', '', f.read())
    m = re.search(r'"extension_modules"\s*:\s*\[(.*?)\]', text, re.S)
    return re.findall(r'"([^"]+)"', m.group(1)) if m else []


def glim_node(context, mode):
    """GLIM node. mode: 'local' (EKF owns odom->base, GLIM owns map->odom)
    or 'gps' (EKF + navsat own map->odom->base, GLIM publishes no TF)."""
    glim_config = LaunchConfiguration('glim_config').perform(context)
    view = is_true(context, 'glim_view')
    params = {'use_sim_time': is_true(context, 'use_sim_time')}

    params['config_path'] = glim_config
    params['odom_child_frame'] = 'base'
    params['map_save_directory'] = '/saved_maps'
    if mode == 'local':
        params['map_odom_mode'] = 'external'
        params['map_odom_planar'] = True
    else:
        params['publish_tf'] = False
    if not view:
        # The viewer needs a display; drop it from the modules (SLAM no longer depends on it)
        modules = [m for m in _extension_modules(glim_config) if m != VIEWER]
        if modules:
            params['extension_modules'] = modules

    return [Node(
        package='glim_ros',
        executable='glim_rosnode',
        output='screen',
        emulate_tty=True,
        # GLIM's viewer runs on the default GPU: on hybrid-GPU hosts (PRIME offload enabled for the
        # container) both variables must be cleared, offload=0 with vendor=nvidia fails (GLXBadFBConfig)
        additional_env={'__NV_PRIME_RENDER_OFFLOAD': '0', '__GLX_VENDOR_LIBRARY_NAME': ''},
        parameters=[params],
    )]


def rtabmap_available():
    try:
        get_package_share_directory('rtabmap_odom')
        return True
    except PackageNotFoundError:
        return False


def visodom_actions(context, include):
    """Include visual odometry only if RTAB-Map is installed (it is optional in the sim image)."""
    if not is_true(context, 'visodom'):
        return []
    if not rtabmap_available():
        return [LogInfo(msg='[jo_navigation] rtabmap_odom not installed: visual odometry disabled '
                            '(build the image with BUILD_RTABMAP=true to enable it)')]
    return [include]
