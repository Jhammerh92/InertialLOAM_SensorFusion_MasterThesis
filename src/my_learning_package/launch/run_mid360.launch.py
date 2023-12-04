from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
# from math import pi, radians
import os.path

from datetime import datetime

## TODO create a live sensor launch file, and use that in record_launch

def generate_launch_description():
    ld = LaunchDescription()

    package_name='my_learning_package' #<--- CHANGE ME IF USED IN ANTOHER PACKAGE

    # recorded_topics = ["/livox/lidar", "/imu/data_raw", "/imu/acceleration", "/imu/angular_velocity", "/imu/mag", "/imu/time_ref"]

    # now = datetime.now()
    # dt_string = now.strftime("%d%m%Y_%H%M%S")

    # config_dir = os.path.join(get_package_share_directory('my_learning_package'), 'config')
    # recordings_dir = r'/home/slamnuc/bagfiles'
    # recording_name = 'lidar_imu_' + dt_string

    # print(f"Recording to folder: {recording_name}")

    lidar_driver_node= IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'ros_MID360_launch.py')]),
    )

    # lidar_group_action_launch = GroupAction(
    #     actions=[
    #         SetRemap(src="/livox/imu", dst="/imu/data_HAP"), # remap the out to something used by filter, remember to do this with RT imeplementation!
    #         lidar_driver_node
    #     ]
    # )

    imu_convert_node = Node(
        package=package_name,
        executable="imu_HAP_converter",
    )


    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # to follow in rviz
    transform_node_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'livox_frame']
    
    )
 
    

    ld.add_action(lidar_driver_node)
    # ld.add_action(lidar_group_action_launch)
    ld.add_action(transform_node_livox)
    ld.add_action(imu_convert_node)

    return ld