from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='logs/encos_bag'),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'sqlite3',
                            '-o', LaunchConfiguration('output'), '--topics', '/encos/control_sample',
                            '/encos/motor_state', '/encos/communication_stats', '/encos/fault',
                            '/encos/raw_tx', '/encos/raw_rx', '/joint_states'], output='screen'),
    ])
