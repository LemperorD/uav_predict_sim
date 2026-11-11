from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    mockamap_node = launch_ros.actions.Node(
        package='mockamap', executable='mockamap_node', 
        name='mockamap_node', output='screen',
        parameters=[{'seed': 510},
                    {'update_freq': 1.0},
                    # box edge length, unit meter
                    {'resolution': 0.1},
                    # map size unit meter
                    {'x_length': 20},
                    {'y_length': 20},
                    {'z_length': 2},
                    {'type': 4},
                    {'numNodes': 64},
                    {'connectivity': 0.5},
                    {'roadRad': 4},
                    {'nodeRad': 3}])

    ld = LaunchDescription()
    ld.add_action(mockamap_node)
    ld.add_action(launch_ros.actions.Node(
        package='prediction_visualize', executable='scene_viewer', output='screen',
        parameters=[{'cloud_topics': ['/mock_map'], 'odometry_topics': ['']}]))

    return ld
