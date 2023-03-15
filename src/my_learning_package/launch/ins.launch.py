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
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    ins_node = Node(
        package="my_learning_package",
        name="ins",
        executable="ins",
        parameters=[os.path.join(config_dir, 'ins.yaml')],
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
    )

    madgwick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','imu_madgwick_filter.launch.py'
        )])
    )



    ld.add_action(ins_node)
    ld.add_action(madgwick_launch)

    return ld