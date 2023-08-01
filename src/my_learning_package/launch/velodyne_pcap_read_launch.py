from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from math import pi, radians

def generate_launch_description():
    ld = LaunchDescription()


    velodyne_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        parameters=[
            {"model":"VLP16"},
            {"frame_id": "velodyne"},
            {"rpm": 600.0},
            {"pcap":r"/home/slamnuc/vlp16_recordings/20230302_145113/VLP16_lidar_ts_01.pcap"}, # hardcoded data path
            # {"pcap":r"/media/slamnuc/HAMMER DATA 2TB/DTU_LIDAR_20220523/20220523_124156/VLP16_lidar_ts_01.pcap"}, # hardcoded data path
            {"read_once":True},
            {"read_fast":False}, ## realtime or as fast as possible. In post-processing it might be advantagoues to do as fast as possible, but still making sure all msg are recieved
            {"cut_angle": 2*pi}
        ]
    )
    

    transform_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # arguments = ['--x', '2.047', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', str(radians(-2.91)), '--roll', str(radians(3.48)), '--frame-id', 'odom', '--child-frame-id', 'velodyne']
            # arguments = [ '0', '0', '2.047', '0', '-0.10157816', '0.12147491' , 'odom', 'velodyne']
            arguments = [ '0', '0', '2.047', '0', '0.05', '0.065' , 'odom', 'velodyne']
            # arguments = [ '0', '0', '2.047', '0.00254445050', '-0.0303232449', '-0.000725556189','0.999215968', 'odom', 'velodyne']
            # arguments = [ '0', '0', '2.047', '0', '0', '0','1', 'odom', 'velodyne']
    )
    # '-0.12147491'

    # velodyne_convert_node = IncludeLaunchDescription(
    #     launch_description_source='/opt/ros/foxy/share/velodyne_pointcloud/launch/velodyne_convert_node-VLP16-launch.py',
    #     # add launch arguments
    # )


    velodyne_convert_launch = GroupAction(
        actions=[
            SetRemap(src="/velodyne_points", dst="/velodyne/lidar"), # ramap the out to something used by filter, remember to do this with RT imeplementation!
            IncludeLaunchDescription(
                launch_description_source='/opt/ros/foxy/share/velodyne_pointcloud/launch/velodyne_convert_node-VLP16-launch.py',
                # add launch arguments
            )
        
        ]
    )


    ld.add_action(velodyne_convert_launch)
    # ld.add_action(velodyne_convert_node)
    ld.add_action(velodyne_node)
    ld.add_action(transform_node)
    return ld