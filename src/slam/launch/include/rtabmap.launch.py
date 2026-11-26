from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchService
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        'queue_size': 50,  
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Grid/RangeMin': '0.2',  
        'Optimizer/GravitySigma': '0',  # Disable imu constraints (we are already in 2D)
        'Grid/Sensor': 'true',
        #'approx_sync_max_interval': 0.02,  
        #'queue_size_imu': 300,  
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', '/depth_cam/rgb0/image_raw'),
        ('rgb/camera_info', '/depth_cam/rgb0/camera_info'),
        ('depth/image', '/depth_cam/depth0/image_raw'),
        ('odom', '/odom'),
        ('scan','/scan_raw'),
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.008, 'use_sim_time': use_sim_time, 'qos': qos}],
            remappings=remappings),

        # SLAM Mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

# sudo apt update
# sudo apt install ros-humble-rtabmap-ros 
# sudo apt install ros-humble-diagnostic-updater 

# To run this in the terminal, run on 1 terminal the syncing:

# ros2 run rtabmap_sync rgbd_sync --ros-args \
#   -p approx_sync:=true \
#   -p approx_sync_max_interval:=0.008 \
#   -p use_sim_time:=false \
#   -p qos:=2 \
#   -r /tf:=tf \
#   -r /tf_static:=tf_static \
#   -r rgb/image:=/depth_cam/rgb0/image_raw \
#   -r rgb/camera_info:=/depth_cam/rgb0/camera_info \
#   -r depth/image:=/depth_cam/depth0/image_raw \
#   -r odom:=/odom \
#   -r scan:=/scan_raw

#   And in terminal 2 the 3D slam:

#   ros2 run rtabmap_slam rtabmap -d --ros-args \
#   -p frame_id:=base_footprint \
#   -p use_sim_time:=false \
#   -p subscribe_rgbd:=true \
#   -p subscribe_scan:=true \
#   -p use_action_for_goal:=true \
#   -p qos_scan:=2 \
#   -p qos_image:=2 \
#   -p qos_imu:=2 \
#   -p queue_size:=50 \
#   -p Reg/Strategy:="'1'" \
#   -p Reg/Force3DoF:="'true'" \
#   -p Grid/RangeMin:="'0.2'" \
#   -p Optimizer/GravitySigma:="'0'" \
#   -p Grid/Sensor:="'true'" \
#   -r /tf:=tf \
#   -r /tf_static:=tf_static \
#   -r rgb/image:=/depth_cam/rgb0/image_raw \
#   -r rgb/camera_info:=/depth_cam/rgb0/camera_info \
#   -r depth/image:=/depth_cam/depth0/image_raw \
#   -r odom:=/odom \
#   -r scan:=/scan_raw