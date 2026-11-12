"""No chassis drivers, ground-truth publisher, EKF or navigation nodes are launched."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wheeltec_vision_benchmark.runtime import estimator_spec, prepare_run


def generate_launch_description():
    def build(context):
        def arg(name):
            return LaunchConfiguration(name).perform(context)
        dataset, output = str(Path(arg('dataset')).expanduser().resolve()), Path(arg('output')).expanduser().resolve()
        run = arg('run_estimator').lower() == 'true'
        mode = arg('mode')
        _, _, config = prepare_run(dataset, output, mode, run)
        player = Node(package='wheeltec_vision_benchmark', executable='euroc_player',
                      name='euroc_player', output='screen', parameters=[dict(
                          dataset=dataset, output_dir=str(output), prepared=True, mode=mode,
                          run_estimator=run, use_sim_time=True,
                          readiness_timeout_s=float(arg('readiness_timeout_s')),
                          drain_seconds=float(arg('drain_seconds')))])
        # Construct all subscriptions before player releases the first sample. Readiness
        # is checked against discovered sensor subscribers and odom-info publishers.
        nodes = [Node(**estimator_spec(config, mode))] if run else []
        stop = RegisterEventHandler(OnProcessExit(target_action=player, on_exit=[
            EmitEvent(event=Shutdown(reason='EuRoC player finished; results saved'))]))
        return [*nodes, stop, player]
    return LaunchDescription([
        DeclareLaunchArgument('dataset'), DeclareLaunchArgument('output'),
        DeclareLaunchArgument('mode', default_value='vio', choices=['stereo', 'vio']),
        DeclareLaunchArgument('run_estimator', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('readiness_timeout_s', default_value='90'),
        DeclareLaunchArgument('drain_seconds', default_value='5'), OpaqueFunction(function=build)])
