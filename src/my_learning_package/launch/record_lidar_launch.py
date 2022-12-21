from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess#, TimerAction



# from ament_index_python.packages import get_package_share_directory
# from math import pi, radians
import os.path

from datetime import datetime

def generate_launch_description():
    ld = LaunchDescription()

    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")

    # config_dir = os.path.join(get_package_share_directory('my_learning_package'), 'config')
    recordings_dir = r'/home/slamnuc/bagfiles'
    recording_name = 'lidar_' + dt_string

    recorded_topics = "/livox/lidar"

    print(f"Recording to folder: {recording_name}")

    # sync_time_ptpd = ExecuteProcess(
    #     cmd=[""]
    # ) 

    lidar_driver_node = IncludeLaunchDescription(
        launch_description_source='/home/slamnuc/ws_livox/src/livox_ros2_driver/launch/livox_lidar_launch.py',
        # add launch arguments
    )
    
    bag_node = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', recorded_topics, '-o', os.path.join(recordings_dir, recording_name)],
            output='screen'
    )
   

    # to follow in rviz
    # transform_node_livox = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'livox_frame']
    
    # )
 
    # TimerAction(period=60.0,
    #         actions=[Node(...), Node(...)]),
    




    ld.add_action(lidar_driver_node)
    # ld.add_action(transform_node_livox)
    ld.add_action(bag_node)

    return ld