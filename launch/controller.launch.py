from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('controller_pkg')

    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'controller_config.yaml'
    ])

    # Launch arguments
    mod_arg = DeclareLaunchArgument(
        'mod',
        default_value='real',
        description='Launch mode: sim (TF mcl_map->ego_racecar/base_link + /ego_racecar/odom), real (TF map->base_link + /odom)'
    )

    # Dynamic odom topic based on mod
    odom_topic = PythonExpression([
        "'/ego_racecar/odom' if '", LaunchConfiguration('mod'), "' == 'sim' else '/odom'"
    ])

    # Dynamic TF map and base_link frames based on mod
    map_frame = PythonExpression([
        "'mcl_map' if '", LaunchConfiguration('mod'), "' == 'sim' else 'map'"
    ])

    base_link_frame = PythonExpression([
        "'ego_racecar/base_link' if '", LaunchConfiguration('mod'), "' == 'sim' else 'base_link'"
    ])

    # Controller node
    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller',
        parameters=[
            config_file,
            {
                'map_frame': map_frame,
                'base_link_frame': base_link_frame,
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/planned_waypoints', '/local_waypoints'),
            ('/odom', odom_topic),
            ('/drive', '/drive'),
        ],
    )

    return LaunchDescription([
        mod_arg,
        controller_node,
    ])
