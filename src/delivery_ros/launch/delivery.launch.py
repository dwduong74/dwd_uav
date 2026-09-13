from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config=os.path.join(get_package_share_directory('delivery_ros'),'config','pi4.yaml')
    cfg=LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=config),
        DeclareLaunchArgument('image_topic',default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic',default_value='/camera/camera_info'),
        Node(package='delivery_ros',executable='vision',name='vision',output='screen',parameters=[cfg],
             remappings=[('camera/image_raw',LaunchConfiguration('image_topic')),
                         ('camera/camera_info',LaunchConfiguration('camera_info_topic'))]),
        Node(package='delivery_ros',executable='px4_adapter',name='px4_adapter',output='screen',parameters=[cfg]),
        Node(package='delivery_ros',executable='payload',name='payload',output='screen',parameters=[cfg]),
        Node(package='delivery_ros',executable='mission',name='mission',output='screen',parameters=[cfg]),
    ])
