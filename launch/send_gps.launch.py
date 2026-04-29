import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def resolve_mission_file(context, *args, **kwargs):
    mission = LaunchConfiguration('mission').perform(context)
    mode    = LaunchConfiguration('mode').perform(context)

    if not mission:
        raise RuntimeError(
            "Argument 'mission' is required.\n"
            "Pass a filename:  mission:=my_mission.yaml\n"
            "Or a full path:   mission:=/absolute/path/to/mission.yaml"
        )

    # Detect whether it's a path or a bare filename
    if os.path.isabs(mission) or os.sep in mission:
        # Looks like a path — use as-is
        mission_path = mission
    else:
        # Bare filename — resolve inside the package's missions/ folder
        pkg_share = get_package_share_directory('jo_navigation')
        mission_path = os.path.join(pkg_share, 'missions', mission)
        # Append .yaml if the user omitted the extension
        if not os.path.splitext(mission_path)[1]:
            mission_path += '.yaml'

    if not os.path.isfile(mission_path):
        raise RuntimeError(f"Mission file not found: {mission_path}")

    node = Node(
        package='jo_navigation',
        executable='send_gps_waypoints.py',
        name='gps_waypoint_follower',
        output='screen',
        emulate_tty=True,
        #prefix='xterm -e',
        parameters=[{
            'mission_file': mission_path,
            'mode': mode,
        }]
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission',
            default_value='',
            description=(
                'Mission file name (e.g. "my_mission.yaml" or "my_mission") '
                'searched inside the package missions/ folder, '
                'or a full absolute path to the file.'
            )
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='auto',
            description='Navigation mode: "auto" (all waypoints at once) or "step" (one at a time).'
        ),
        OpaqueFunction(function=resolve_mission_file),
    ])