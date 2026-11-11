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
                    {'type': 3},
                    {'road_width': 0.5},
                    {'add_wall_x': 0},
                    {'add_wall_y': 0},
                    # maze type: 1 recursive division maze
                    {'maze_type': 1}]
    )

    ld = LaunchDescription()
    ld.add_action(mockamap_node)
    ld.add_action(launch_ros.actions.Node(
        package='prediction_visualize', executable='scene_viewer', output='screen',
        parameters=[{'cloud_topics': ['/mock_map'], 'odometry_topics': ['']}]))

    return ld
