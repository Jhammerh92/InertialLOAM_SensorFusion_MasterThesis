from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
# from math import pi, radians
import os.path

from datetime import datetime

def generate_launch_description():
    ld = LaunchDescription()

    package_name='my_learning_package' #<--- CHANGE ME

    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")

    # config_dir = os.path.join(get_package_share_directory('my_learning_package'), 'config')
    recordings_dir = r'/home/slamnuc/bagfiles'
    recording_name = 'lidar_imu_' + dt_string

    lidar_driver_node= IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('livox_ros2_driver'), 'launch', 'livox_lidar_launch.py')]),
    )

    imu_driver_node= IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('bluespace_ai_xsens_mti_driver'), 'launch', 'xsens_mti_node.launch.py')]),
    )

    # lidar_driver_node = IncludeLaunchDescription(
    #     launch_description_source='/home/slamnuc/ws_livox/src/livox_ros2_driver/launch/livox_lidar_launch.py',
    #     # add launch arguments
    # )
    
    bag_node = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', os.path.join(recordings_dir, recording_name)],
            output='screen'
    )
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # to follow in rviz
    transform_node_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'livox_frame']
    
    )
 
    

    ld.add_action(lidar_driver_node)
    ld.add_action(imu_driver_node)
    ld.add_action(transform_node_livox)
    ld.add_action(bag_node)

    return ld