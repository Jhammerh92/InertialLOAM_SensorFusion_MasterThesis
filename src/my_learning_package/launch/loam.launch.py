from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from math import pi, radians
import os.path

def generate_launch_description():
    ld = LaunchDescription()

    package_name='my_learning_package' #<--- CHANGE ME
    # config_dir = os.path.join(get_package_share_directory(package_name), 'config')


    lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','lidar_odometry.launch.py'
        )])
    )


    preprocessing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','preprocessing.launch.py'
        )])
    )


    backend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','backend.launch.py'
        )])
    )
    
    # ekf_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name),'launch','ekf.launch.py'
    #     )])
    # )

    # rsp -> robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )])
                , launch_arguments={'use_sim_time': 'false'}.items()
    )

    madgwick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','imu_madgwick_filter.launch.py'
        )])
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
        arguments = [ '0.021', '0', '-0.192', '0', '0', '0', 'long_term_map', 'odom']
    )


    transform_base_link_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '-0.064', '0', '0', '0' , 'base_link', 'base_link_footprint']
    )


    # transform_node_livox = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0.050', '0', '-0.205', '0', '0', '0' , 'odom', 'base_link']  
    # )

    # transform_node_preproc = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'lidar_preproc']
    # 
    # )

    
    # transform_node_odom_lidar_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '-0.021', '0', '0.192', '0', '0', '0' , 'odom', 'lidar_odom'] 
    # )


    # transform_node_base_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '-0021', '0', '0.192', '0', '0', '0' , 'base_link', 'livox_frame']
    # )

    # transform_node_livox_to_lidar_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [ '0', '0', '0', '0', '0', '0' , 'livox_frame', 'lidar_odom']  # this is wrong lidar_odom should be static to odom and  not following the robot frame static ego-frame ie. livox_frame
    # )
    



    ld.add_action(madgwick_launch)
    ld.add_action(preprocessing_launch)
    ld.add_action(backend_launch)
    # ld.add_action(ekf_launch)
    ld.add_action(lidar_odometry_launch)

    ld.add_action(transform_node_global_map_lidar_odom)
    ld.add_action(transform_node_global_map_odom)
    # ld.add_action(transform_node_long_term_map_odom)
    ld.add_action(transform_base_link_footprint)

    # ld.add_action(transform_node_odom)
    # ld.add_action(transform_node_odom_lidar_odom)
    # ld.add_action(rsp)
    
    return ld