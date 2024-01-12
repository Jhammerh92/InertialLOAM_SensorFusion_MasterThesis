from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from math import pi, radians
import os.path

def generate_launch_description():
    ld = LaunchDescription()


    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'INS.rviz')]
    )
    # print('-d ' +os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz'))

    # transform_node_livox = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'base_link']
    # )

    # transform_node_preproc = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'lidar_preproc']
    
    # )
    
    transform_node_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '-0.021', '0', '0.192', '0', '0', '0' , 'odom', 'livox_frame']
    )

    transform_node_global_map_lidar_odom = Node( # this should change in the future if the map is ever going to be firm and disconnected from odometry
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0' , 'global_map', 'lidar_odom']
    )

    transform_node_global_map_odom = Node( # this should change in the future if the map is ever going to be firm and disconnected from odometry
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0.021', '0', '-0.192', '0', '0', '0', 'global_map', 'odom']
    )

    transform_node_long_term_map_odom = Node( # this should change in the future if the map is ever going to be firm and disconnected from odometry
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '0', '0', '0', '0', 'global_map', 'long_term_map']
    )

    # transform_node_global_map = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'global_map', 'odom']
    # )
    




    ld.add_action(rviz2_node)
    ld.add_action(transform_node_global_map_lidar_odom)
    ld.add_action(transform_node_global_map_odom)
    ld.add_action(transform_node_long_term_map_odom)
    # ld.add_action(transform_node_livox)
    # ld.add_action(transform_node_preproc)
    ld.add_action(transform_node_odom)
    # ld.add_action(transform_node_global_map)
    return ld