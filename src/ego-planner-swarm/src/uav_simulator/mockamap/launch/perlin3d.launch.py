
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    mockamap_node = launch_ros.actions.Node(
            package='mockamap', executable='mockamap_node',
            output='screen',
            parameters=[{'seed': 511},
                        {'update_freq': 1.0},
                        # box edge length, unit meter
                        {'resolution': 0.15},
                        # map size unit meter
                        {'x_length': 40},
                        {'y_length': 20},
                        {'z_length': 5},
                        # 1 perlin noise 3D, 2 perlin box random map, 3 2d maze still developing
                        {'type': 1},
                        # 1 perlin noise parameters
                        # complexity:  base noise frequency,large value will be complex. typical 0.0 ~ 0.5
                        # fill:        infill persentage. typical: 0.4 ~ 0.0
                        # fractal:     large value will have more detail
                        # attenuation: for fractal attenuation. typical: 0.0 ~ 0.5
                        {'complexity': 0.07},
                        {'fill': 0.1},
                        {'fractal': 1},
                        {'attenuation': 0.1}])

    ld = LaunchDescription()
    ld.add_action(mockamap_node)
    ld.add_action(launch_ros.actions.Node(
        package='prediction_visualize', executable='scene_viewer', output='screen',
        parameters=[{'cloud_topics': ['/mock_map'], 'odometry_topics': ['']}]))

    return ld
