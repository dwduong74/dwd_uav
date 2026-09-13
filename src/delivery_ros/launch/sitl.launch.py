"""Run after PX4/Gazebo/Agent. Requires ros_gz_bridge for Gazebo Harmonic."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',output='screen',
             arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                        '/camera/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
             parameters=[{'override_timestamps_with_wall_time':True}]),
        Node(package='tf2_ros',executable='static_transform_publisher',
             arguments=['--x','0','--y','0','--z','-0.05','--qx','0.7071067811865476',
                        '--qy','-0.7071067811865476','--qz','0','--qw','0',
                        '--frame-id','base_link','--child-frame-id','camera_optical_frame']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('delivery_ros'),'launch','delivery.launch.py')),
            launch_arguments={'config':LaunchConfiguration('config')}.items()),
    ])
