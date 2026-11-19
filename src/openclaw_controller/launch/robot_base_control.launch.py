import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context):
    compiled = os.environ['need_compile']

    if compiled == 'True':
        openclaw_controller_package_path = get_package_share_directory('openclaw_controller')
    else:
        openclaw_controller_package_path = '/home/ubuntu/ros2_ws/src/openclaw_controller'

    device_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(openclaw_controller_package_path, 'launch/include/device_base.launch.py')),
    )

    use_asr = LaunchConfiguration('use_asr', default='false')
    use_asr_arg = DeclareLaunchArgument('use_asr', default_value=use_asr)
    voice_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(openclaw_controller_package_path, 'launch/voice_control.launch.py')),
        launch_arguments={
            'use_asr': use_asr,
        }.items(),
    )

    claw_move_control_node = Node(
        package='openclaw_controller',
        executable='claw_move_control',
        output='screen',
    )

    use_tts = LaunchConfiguration('use_tts', default='true')
    use_tts_arg = DeclareLaunchArgument('use_tts', default_value=use_tts)
    claw_tts_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(openclaw_controller_package_path, 'launch/include/claw_tts.launch.py')),
        launch_arguments={
            'use_tts': use_tts,
        }.items(),
    )

    return [
        use_tts_arg,
        use_asr_arg,
        device_base_launch,
        claw_move_control_node,
        claw_tts_launch,
        voice_control_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
