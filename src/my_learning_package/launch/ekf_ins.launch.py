from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from math import pi, radians
import os.path


def generate_launch_description():
    ld = LaunchDescription()

    package_name='my_learning_package' #<--- CHANGE ME
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    ekf_node = Node(
        package="my_learning_package",
        name="ekf_ins",
        executable="ekf_ins",
        parameters=[os.path.join(config_dir, 'ekf_ins.yaml')],
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
    )



    ld.add_action(ekf_node)

    return ld