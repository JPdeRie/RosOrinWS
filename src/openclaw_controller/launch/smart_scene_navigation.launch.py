import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction


def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        openclaw_controller_package_path = get_package_share_directory('openclaw_controller')
    else:
        openclaw_controller_package_path = '/home/ubuntu/ros2_ws/src/openclaw_controller'

    use_tts = LaunchConfiguration('use_tts', default='true')
    use_tts_arg = DeclareLaunchArgument('use_tts', default_value=use_tts)
    claw_tts_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(openclaw_controller_package_path, 'launch/include/claw_tts.launch.py')),
        launch_arguments={
            'use_tts': use_tts,
        }.items(),
    )

    camera_topic_value = '/depth_cam/rgb0/image_raw'
    camera_topic = LaunchConfiguration('camera_topic', default=camera_topic_value)
    road_network_map = LaunchConfiguration('road_network_map', default='road_network_community').perform(context)
    road_network_map_arg = DeclareLaunchArgument('road_network_map', default_value=road_network_map)

    road_network_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('large_models_examples'),
                'large_models_examples/road_network/road_network.launch.py'
            )
        ),
        launch_arguments={
            'camera_topic': camera_topic,
            'file_name': road_network_map,
            'use_yolo_detect': 'false',
        }.items(),
    )

    navigation_mode = LaunchConfiguration('navigation_mode', default='normal')
    navigation_mode_arg = DeclareLaunchArgument('navigation_mode', default_value=navigation_mode)
    claw_navigation_manager_node = Node(
        package='openclaw_controller',
        executable='claw_navigation_manager',
        output='screen',
        parameters=[
            {
                'navigation_mode': navigation_mode,
            }
        ],
    )

    return [
        use_tts_arg,
        navigation_mode_arg,
        GroupAction(
            actions=[
                claw_tts_launch,
                claw_navigation_manager_node,
                road_network_map_arg,
                road_network_launch,
            ],
        )
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
