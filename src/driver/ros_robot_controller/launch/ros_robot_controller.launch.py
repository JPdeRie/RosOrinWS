from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)
    servo_config_path = LaunchConfiguration(
        'servo_config_path',
        default='/home/ubuntu/software/Servo_upper_computer/servo_config.yaml')
    servo_config_path_arg = DeclareLaunchArgument(
        'servo_config_path', default_value=servo_config_path)

    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        namespace=namespace,
        output='screen',
        parameters=[{
            'imu_frame': imu_frame,
            'servo_config_path': servo_config_path,
        }]
    )

    return LaunchDescription([
        namespace_arg,
        imu_frame_arg,
        servo_config_path_arg,
        ros_robot_controller_node
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object. (创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
