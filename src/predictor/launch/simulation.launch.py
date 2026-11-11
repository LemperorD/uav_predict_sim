"""EGO kinematic simulator -> noisy observation -> predictor -> Pangolin."""
from pathlib import Path
import csv
import math
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def initial_position(params):
    """Start the simulator at the first command, including phase and CSV input."""
    if params.get('input_csv'):
        with Path(params['input_csv']).open() as stream:
            first = next(csv.DictReader(stream))
        return [float(first[key]) for key in ('px', 'py', 'pz')]
    position = list(params.get('trajectory_origin', [0.0, 0.0, 2.0]))
    trajectory = params.get('trajectory', 'figure8')
    if trajectory in ('circle', 'helix'):
        radius = params.get('trajectory_radius', 4.0)
        phase = params.get('trajectory_circle_phase', 0.0)
        position[0] += radius * math.cos(phase)
        position[1] += radius * math.sin(phase)
    elif trajectory in ('figure8', 'sinusoid'):
        amplitude = params.get('trajectory_amplitude', [4.0, 3.0, 0.6])
        phase = params.get('trajectory_phase', [0.0, 0.0, 0.0])
        position = [position[i] + amplitude[i] * math.sin(phase[i]) for i in range(3)]
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        raise ValueError('initial trajectory position must contain three finite values')
    return position


def setup(context):
    config = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    visualization_config = Path(
        LaunchConfiguration('visualization_config').perform(context)).expanduser().resolve()
    style = yaml.safe_load(visualization_config.read_text())['prediction_viewer']['ros__parameters']
    if isinstance(style, dict) and style.get('drone_mesh_file'):
        mesh_file = Path(style['drone_mesh_file']).expanduser()
        style['drone_mesh_file'] = str((visualization_config.parent / mesh_file).resolve()
                                      if not mesh_file.is_absolute() else mesh_file)
    if not isinstance(style, dict):
        raise ValueError('visualization config must contain prediction_viewer.ros__parameters')
    for key in ('output_dir', 'input_csv', 'ffmpeg_path'):
        if params.get(key):
            p = Path(params[key]).expanduser()
            params[key] = str((config.parent / p).resolve() if not p.is_absolute() else p)
    override = LaunchConfiguration('viewer').perform(context)
    if override != 'config':
        params['show_viewer'] = override == 'true'
    for key in ('duration', 'sample_rate', 'prediction_step'):
        if not math.isfinite(params[key]) or params[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    for key in ('window_width', 'window_height'):
        if type(params[key]) is not int or params[key] <= 0:
            raise ValueError(f'{key} must be a positive integer')
    output = Path(params['output_dir'])
    output.mkdir(parents=True, exist_ok=True)
    # Snapshot of resolved parameters makes each run reproducible.
    snapshot = Path(params.get('snapshot_file', 'resolved.yaml'))
    if not snapshot.is_absolute():
        snapshot = output / snapshot
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    # Runner-only lists may be empty, which ROS launch cannot infer as a typed array.
    runner_keys = {'offline_plot', 'plot_label', 'baseline_samples', 'baseline_labels',
                   'comparison_metrics_file'}
    ros_params = {key: value for key, value in params.items() if key not in runner_keys}
    viewer_params = {**ros_params, **style}
    snapshot.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': params},
        'prediction_viewer': {'ros__parameters': viewer_params},
    }))
    topics = [('odometry', params['odometry_topic']), ('command', params['command_topic'])]
    start = initial_position(params)
    simulator = Node(package='poscmd_2_odom', executable='poscmd_2_odom',
                     name='ego_kinematic_simulator', output='screen',
                     parameters=[dict(zip(('init_x', 'init_y', 'init_z'), start))], remappings=topics)
    predictor = Node(package='predictor', executable='predictor_node',
                     output='screen', parameters=[ros_params], remappings=topics)
    source = Node(package='predictor', executable='kinematic_source',
                  output='screen', parameters=[ros_params], remappings=topics)
    nodes = [predictor]
    processors = [simulator, predictor]
    if params.get('show_viewer', True):
        viewer = Node(package='prediction_visualize', executable='prediction_viewer',
                      output='screen', parameters=[viewer_params],
                      remappings=[('odometry', params['odometry_topic'])])
        nodes.append(viewer)
        processors.append(viewer)
    # Subscribers and GL initialization start before commands are sent.
    nodes.append(TimerAction(period=2.0, actions=[simulator, source]))
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=source, on_exit=[
        TimerAction(period=0.5, actions=[EmitEvent(event=Shutdown(reason='trajectory completed'))])
    ])))
    # Fail fast when a processing node dies before the source finishes.
    for node in processors:
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            EmitEvent(event=Shutdown(reason='processing node exited'))
        ])))
    return nodes


def generate_launch_description():
    default = str(Path(get_package_share_directory('predictor')) / 'config' / 'simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('visualization_config', default_value=str(
            Path(get_package_share_directory('predictor')) / 'config' / 'visualization.yaml')),
        DeclareLaunchArgument('viewer', default_value='config', choices=['config', 'true', 'false']),
        OpaqueFunction(function=setup),
    ])
