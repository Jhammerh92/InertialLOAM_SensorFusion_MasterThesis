from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from math import pi, radians
import os.path

def generate_launch_description():
    ld = LaunchDescription()

    # config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config', 'imu_filter_odom.yaml')


    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'IMU2.rviz')]
    )
    # print('-d ' +os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz'))

    transform_node_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'imu_link']
    )
    transform_node_imu_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'imu_link', 'livox_frame']
    )
    # ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros.args -p use_mag:=false -p gain:= 0.9
    # madgwick_filter_node = Node(
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',
    #     name='imu_filter',
    #     parameters=[config_dir],
    #     # arguments = ['use_mag', 'false', 'gain', '0.9', 'constant_dt','0.0025', 'remove_gravity_vector', 'true']
    
    # )

    madgwick_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('my_learning_package'),'launch','imu_madgwick_filter.launch.py'
        )])
    )
    
    # transform_node_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'lidar_odom']
    # )

    # transform_node_global_map = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'global_map']
    # )
    




    ld.add_action(rviz2_node)
    ld.add_action(transform_node_imu)
    ld.add_action(transform_node_imu_to_lidar)
    ld.add_action(madgwick_filter_launch)
    # ld.add_action(transform_node_preproc)
    # ld.add_action(transform_node_odom)
    # ld.add_action(transform_node_global_map)
    return ld