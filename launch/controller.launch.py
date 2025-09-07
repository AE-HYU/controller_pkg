from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
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
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Use simulation mode (ego_racecar/odom) if true, real car mode (/pf/pose/odom) if false'
    )
    
    # Controller node for simulation mode
    controller_sim_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/planned_path', '/planned_path'),
            # ('/odom', '/ego_racecar/odom'),
            ('/odom', '/pf/pose/odom'), # MCL odom
            ('/drive', '/drive'),
        ],
        condition=IfCondition(LaunchConfiguration('sim_mode'))
    )
    
    # Controller node for real car mode
    controller_real_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/planned_path', '/planned_path'),
            ('/odom', '/pf/pose/odom'),
            ('/drive', '/drive'),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim_mode'))
    )
    
    return LaunchDescription([
        sim_mode_arg,
        controller_sim_node,
        controller_real_node,
    ])
