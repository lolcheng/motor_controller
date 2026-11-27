from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='data/encos_bag'),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'sqlite3',
                            '-o', LaunchConfiguration('output'), '--topics', '/encos/raw_rx',
                            '/encos/motor_state'], output='screen'),
    ])
