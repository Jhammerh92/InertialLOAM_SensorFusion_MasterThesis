from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from math import pi, radians
import os.path

def generate_launch_description():
    ld = LaunchDescription()


    lidar_odometry_node = Node(
        package="my_learning_package",
        executable="lidar_odometry_velodyne",
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
    )
    preprocessing_node = Node(
        package="my_learning_package",
        executable="preprocessing_velodyne",
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
    )
    backend_node = Node(
        package="my_learning_package",
        executable="loam_backend",
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
    )





    transform_node_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'velodyne'] 
    )

    transform_node_preproc = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'lidar_preproc'] 
    )
    
    transform_node_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'lidar_odom']
    )

    transform_node_global_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'global_map']
    )
    




    ld.add_action(preprocessing_node)
    ld.add_action(backend_node)
    ld.add_action(lidar_odometry_node)
    ld.add_action(transform_node_velodyne)
    ld.add_action(transform_node_preproc)
    ld.add_action(transform_node_odom)
    ld.add_action(transform_node_global_map)
    return ld