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

    # package_name='my_learning_package' #<--- CHANGE ME IF USED IN ANTOHER PACKAGE

    imu_launch = GroupAction(
        actions=[
            SetRemap(src="/imu/data", dst="/imu/data_raw"), # ramap the out to something used by filter, remember to do this with RT imeplementation!

            IncludeLaunchDescription( # IMU driver launch
                        PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory('bluespace_ai_xsens_mti_driver'), 'launch', 'xsens_mti_node.launch.py')]),
            ),

        ]
    )


    ld.add_action(imu_launch)

    # ld.add_action(bag_node)

    return ld