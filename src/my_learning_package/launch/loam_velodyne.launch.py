from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from math import pi, radians
import os.path


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
            get_package_share_directory(package_name),'launch','lidar_odometry_velodyne.launch.py'
        )])
    )


    preprocessing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','preprocessing_velodyne.launch.py'
        )])
    )


    backend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','backend.launch.py'
        )])
    )


    # # rsp -> robot state publisher
    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rsp.launch.py'
    #             )])
    #             , launch_arguments={'use_sim_time': 'false'}.items()
    # )

  

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


    transform_base_link_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [ '0', '0', '-0.064', '0', '0', '0' , 'base_link', 'base_link_footprint']
    )





    ld.add_action(preprocessing_launch)
    ld.add_action(backend_launch)

    ld.add_action(lidar_odometry_launch)

    ld.add_action(transform_node_global_map_lidar_odom)
    ld.add_action(transform_node_global_map_odom)
    ld.add_action(transform_base_link_footprint)
    return ld


# def generate_launch_description():
#     ld = LaunchDescription()


#     lidar_odometry_node = Node(
#         package="my_learning_package",
#         executable="lidar_odometry_velodyne",
#         # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
#     )
#     preprocessing_node = Node(
#         package="my_learning_package",
#         executable="preprocessing_velodyne",
#         # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
#     )
#     backend_node = Node(
#         package="my_learning_package",
#         executable="loam_backend",
#         # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
#     )





#     transform_node_velodyne = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'velodyne'] 
#     )

#     transform_node_preproc = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'lidar_preproc'] 
#     )
    
#     transform_node_odom = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'lidar_odom']
#     )

#     transform_node_global_map = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments = [ '0', '0', '0', '0', '0', '0' , 'odom', 'global_map']
#     )
    




#     ld.add_action(preprocessing_node)
#     ld.add_action(backend_node)
#     ld.add_action(lidar_odometry_node)
#     ld.add_action(transform_node_velodyne)
#     ld.add_action(transform_node_preproc)
#     ld.add_action(transform_node_odom)
#     ld.add_action(transform_node_global_map)
#     return ld