from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0', description='Joystick device')
    use_joy_arg = DeclareLaunchArgument('use_joy', default_value='false', description='Use joystick controller (true) or keyboard (false)')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_name': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,
            'scale_linear.x': 0.6,
            'axis_linear.y': 0,
            'scale_linear.y': 0.6,
            'axis_angular.yaw': 3,
            'scale_angular.yaw': 1.5,
            'enable_button': 5, # R1 shoulder button as deadman switch
        }],
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node',
        output='screen',
        prefix='xterm -e',
        condition=UnlessCondition(LaunchConfiguration('use_joy'))
    )

    return LaunchDescription([
        joy_dev_arg,
        use_joy_arg,
        joy_node,
        teleop_joy_node,
        teleop_keyboard_node
    ])
