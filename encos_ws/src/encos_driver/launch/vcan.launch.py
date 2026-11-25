from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    interface = LaunchConfiguration('interface')
    fd = LaunchConfiguration('fd')
    frequency = LaunchConfiguration('frequency')
    motor_id = LaunchConfiguration('motor_id')
    share = FindPackageShare('encos_driver')
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='vcan0'),
        DeclareLaunchArgument('fd', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('frequency', default_value='400'),
        DeclareLaunchArgument('motor_id', default_value='1'),
        DeclareLaunchArgument('virtual_motor', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('raw', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_writes', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('additional_virtual_modes', default_value='false', choices=['true', 'false']),
        ExecuteProcess(
            cmd=[PathJoinSubstitution([FindPackagePrefix('encos_tools'), 'lib', 'encos_tools',
                                       'encos_virtual_motor']),
                 interface, PythonExpression(["'fd' if '", fd, "' == 'true' else 'classic'"]),
                 '--bootstrap-beacon', '--id', motor_id],
            output='screen', condition=IfCondition(LaunchConfiguration('virtual_motor'))),
        Node(
            package='encos_driver', executable='encos_driver_node', output='screen',
            parameters=[
                PathJoinSubstitution([share, 'config', 'vcan.yaml']),
                PathJoinSubstitution([share, 'config', 'EC-A10825-P2-24H-B.yaml']),
                {'can_interface': interface, 'fd_brs': ParameterValue(fd, value_type=bool),
                 'motor_id': ParameterValue(motor_id, value_type=int),
                 'control_hz': ParameterValue(frequency, value_type=int),
                 'publish_raw': ParameterValue(LaunchConfiguration('raw'), value_type=bool),
                 'allow_virtual_writes': ParameterValue(LaunchConfiguration('allow_writes'), value_type=bool),
                 **{name: ParameterValue(LaunchConfiguration('additional_virtual_modes'), value_type=bool)
                    for name in ['allow_virtual_current_torque', 'allow_virtual_braking',
                                 'allow_virtual_electromagnetic_brake']}},
            ]),
    ])
