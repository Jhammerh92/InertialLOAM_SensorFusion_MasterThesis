from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from math import pi, radians
import os.path


def generate_launch_description():
    ld = LaunchDescription()

    package_name='imu_filter_madgwick' #<--- CHANGE ME
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    ekf_node = Node(
        package=package_name,
        name="imu_madgwick_filter",
        executable="imu_filter_madgwick_node",
        parameters=[os.path.join(config_dir, 'imu_filter_odom.yaml'),{"use_mag" : True}],
        # arguments=['-p','use_mag:=false'],
        remappings=[('/imu/data_raw','/imu/data_filter')],
        
    )



    ld.add_action(ekf_node)

    return ld