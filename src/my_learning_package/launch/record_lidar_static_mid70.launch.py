from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
# from math import pi, radians
import os.path



def generate_launch_description():
    ld = LaunchDescription()

    package_name='my_learning_package' #<--- CHANGE ME IF USED IN ANTOHER PACKAGE
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')


    lidar_driver_node= IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('livox_ros2_driver'), 'launch', 'livox_lidar_launch.py')]),
    )

    static_record_node = Node(
        package="my_learning_package",
        name="static_recorder",
        executable="static_recorder",
        parameters=[os.path.join(config_dir, 'static_recorder.yaml')],
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
    )


    ld.add_action(lidar_driver_node)
    ld.add_action(static_record_node)

    return ld