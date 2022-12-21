import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_learning_package' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    rviz = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'robot_viz.rviz')],
                        output='screen')


    jsp_gui = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                        # arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'robot_viz.rviz')],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        rsp,
        rviz,
        jsp_gui,
    ])