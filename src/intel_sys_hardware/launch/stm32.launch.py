from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/rrc', description='STM32 serial port device')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='1000000', description='Serial baudrate')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value='imu_link', description='IMU frame ID')
    # One switch for which end is the front, used by both the controller and wheel odometry.
    front_arg = DeclareLaunchArgument(
        'forward_is_sensor_side', default_value='true',
        description='true: +x in /cmd_vel and odometry is the sensor side (LiDAR/camera); '
                    'false: the STM32 motor convention front')

    stm32_bridge_node = Node(
        package='intel_sys_hardware',
        executable='stm32_bridge',
        name='ros_robot_controller',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'imu_frame': LaunchConfiguration('imu_frame'),
        }]
    )

    mecanum_controller_node = Node(
        package='intel_sys_hardware',
        executable='mecanum_controller',
        name='mecanum_velocity_controller',
        output='screen',
        parameters=[{
            'cmd_vel_topic': 'cmd_vel',
            'motor_topic': 'set_motor',
            'forward_is_sensor_side': ParameterValue(LaunchConfiguration('forward_is_sensor_side'), value_type=bool),
        }]
    )

    odom_publisher_node = Node(
        package='intel_sys_hardware',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{
            'odom_topic': '/odom/wheel',
            'motor_topic': 'set_motor',
            'cmd_vel_topic': 'cmd_vel',
            'publish_tf': False,
            'forward_is_sensor_side': ParameterValue(LaunchConfiguration('forward_is_sensor_side'), value_type=bool),
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        imu_frame_arg,
        front_arg,
        stm32_bridge_node,
        mecanum_controller_node,
        odom_publisher_node
    ])
