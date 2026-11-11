"""One Pangolin scene for a single drone or a swarm in world coordinates."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    count = int(LaunchConfiguration('drone_count').perform(context))
    if count <= 0 or count > 1000:
        raise ValueError('drone_count must be in [1, 1000]')
    return [Node(
        package='prediction_visualize', executable='scene_viewer', output='screen',
        parameters=[{
            'odometry_topics': [f'/drone_{i}_visual_slam/odom' for i in range(count)],
            'cloud_topics': [LaunchConfiguration('cloud_topic').perform(context)],
        }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_count', default_value='1'),
        DeclareLaunchArgument('cloud_topic', default_value='/map_generator/global_cloud'),
        OpaqueFunction(function=setup),
    ])
