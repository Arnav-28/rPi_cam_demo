from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    # Camera node with specific parameters
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[{
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link_optical'
        }]
    )

    return LaunchDescription([

        camera_node,
    ])