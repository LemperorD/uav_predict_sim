"""EGO kinematic simulator -> noisy observation -> IMM -> Pangolin (no RViz)."""
from pathlib import Path
import math
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    config = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
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
    if params['window_width'] % 2 or params['window_height'] % 2:
        raise ValueError('Video dimensions must be even')
    output = Path(params['output_dir'])
    output.mkdir(parents=True, exist_ok=True)
    # Snapshot of resolved parameters makes each run reproducible.
    snapshot = Path(params.get('snapshot_file', 'resolved.yaml'))
    if not snapshot.is_absolute():
        snapshot = output / snapshot
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    snapshot.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    topics = [('odometry', params['odometry_topic']), ('command', params['command_topic'])]
    simulator = Node(package='poscmd_2_odom', executable='poscmd_2_odom',
                     name='ego_kinematic_simulator', output='screen',
                     parameters=[{'init_x': 0.0, 'init_y': 0.0, 'init_z': 2.0}], remappings=topics)
    predictor = Node(package='predictor', executable='predictor_node',
                     output='screen', parameters=[params], remappings=topics)
    source = Node(package='predictor', executable='kinematic_source',
                  output='screen', parameters=[params], remappings=topics)
    nodes = [predictor]
    processors = [simulator, predictor]
    if params.get('show_viewer', True):
        viewer = Node(package='prediction_visualize', executable='prediction_viewer',
                      output='screen', parameters=[params])
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
        DeclareLaunchArgument('viewer', default_value='config', choices=['config', 'true', 'false']),
        OpaqueFunction(function=setup),
    ])
