"""Independent simulation composition, reusing the production safety stack."""
import json,os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix
from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node,SetParameter
from .model import Geometry,write_model,robot_urdf
from .configuration import effective_runtime,ekf_parameters,simulation_time
from wheeltec_vision_nav.launch_common import package_file
from wheeltec_vision_nav.launch_nodes import (rtabmap_node,
    ekf_inputs_node,depth_obstacle_nodes,vo_watchdog_node,navigation_nodes)
from wheeltec_vision_nav.launch_utils import write_visual_nav_params
from wheeltec_vision_nav.nav_params import bundled_runtime_config
from wheeltec_vision_nav.runtime_config import safety_parameters
from .leases import acquire_simulation_leases
from wheeltec_vision_nav.launch_pipeline import supervised_processes


def bridge_specs():
    specs=[('/sim/clock','rosgraph_msgs/msg/Clock','ignition.msgs.Clock','gz_to_ros'),
           ('/cmd_vel_safe','geometry_msgs/msg/Twist','ignition.msgs.Twist','ros_to_gz'),
           ('/sim/actuator/drop_commands','std_msgs/msg/Bool','ignition.msgs.Boolean','ros_to_gz'),
           ('/sim/joints','sensor_msgs/msg/JointState','ignition.msgs.Model','gz_to_ros'),
           ('/sim/ground_truth','nav_msgs/msg/Odometry','ignition.msgs.Odometry','gz_to_ros'),
           ('/sim/raw/imu','sensor_msgs/msg/Imu','ignition.msgs.IMU','gz_to_ros')]
    specs.append(('/sim/contacts','ros_gz_interfaces/msg/Contacts','ignition.msgs.Contacts','gz_to_ros'))
    for name in ('rear_left','rear_right','front_left','front_right'):
        specs.append(('/sim/contacts/'+name,'ros_gz_interfaces/msg/Contacts','ignition.msgs.Contacts','gz_to_ros'))
    for side in ('left','right'):
        specs.extend([(f'/sim/raw/{side}/image','sensor_msgs/msg/Image','ignition.msgs.Image','gz_to_ros'),
                      (f'/sim/raw/{side}/camera_info','sensor_msgs/msg/CameraInfo','ignition.msgs.CameraInfo','gz_to_ros')])
    specs.append(('/sim/raw/left/depth_image','sensor_msgs/msg/Image','ignition.msgs.Image','gz_to_ros'))
    return [topic+'@'+ros+('[' if direction=='gz_to_ros' else ']')+gz for topic,ros,gz,direction in specs]


def bridge_remappings():
    return [('/sim/clock','/clock'),('/sim/joints','/joint_states')]


def vins_config(directory,g,mode):
    from wheeltec_vision_nav.vins_config import write_opencv,matrix
    from .extrinsics import transforms
    import numpy as np
    directory=Path(directory);directory.mkdir()
    (directory/'solver_output').mkdir()
    for side in ('left','right'):
        write_opencv(directory/(side+'.yaml'),dict(model_type='PINHOLE',camera_name=side,
            image_width=g.width_px,image_height=g.height_px,
            distortion_parameters=dict(k1=0.,k2=0.,p1=0.,p2=0.),
            projection_parameters=dict(fx=g.fx,fy=g.fx,cx=(g.width_px-1)/2,cy=(g.height_px-1)/2)))
    tf=transforms(g)
    matrices=[matrix((np.linalg.inv(tf['imu'])@tf[side]).tolist()) for side in ('left','right')]
    write_opencv(directory/'vins.yaml',dict(imu=int(mode=='vio'),num_of_cam=2,
        cam0_calib='left.yaml',cam1_calib='right.yaml',image_width=g.width_px,image_height=g.height_px,
        body_T_cam0=matrices[0],body_T_cam1=matrices[1],estimate_extrinsic=0,multiple_thread=0,
        max_cnt=200,min_dist=20,show_track=0,flow_back=1,max_solver_time=.04,max_num_iterations=8,
        keyframe_parallax=10.,estimate_td=0,td=0.,acc_n=g.accel_noise,gyr_n=g.gyro_noise,
        acc_w=1e-5,gyr_w=1e-6,g_norm=9.81,output_path=str(directory/'solver_output')))
    return str(directory/'vins.yaml')


def build(mode,estimator,database,output,headless=True,reverse=False,geometry_file=''):
    if mode not in ('mapping','localization') or estimator not in ('baseline','stereo','vio'):raise ValueError('invalid mode/estimator')
    database=Path(database).expanduser().resolve()
    database.parent.mkdir(parents=True,exist_ok=True)
    if mode=='localization' and not database.is_file():raise FileNotFoundError('mapping database required before localization')
    output=Path(output).expanduser().resolve();output.mkdir(parents=True,exist_ok=False)
    g=Geometry(**(yaml.safe_load(Path(geometry_file).read_text()) if geometry_file else {}))
    sdf,_=write_model(output,g)
    cfg=effective_runtime(bundled_runtime_config(),g,reverse)
    safe=safety_parameters(cfg,localization=mode=='localization',odom_source='ekf',supervised_reverse=reverse)
    if estimator=='vio':
        # Monitor raw critical inputs even though correlated VIO/IMU loss cannot
        # authorize wheel backup. The EKF still excludes independent IMU fusion.
        safe.update(wheel_backup=False,require_wheel=True,require_imu=True)
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='sim_bridge',output='screen',
                arguments=bridge_specs(),remappings=bridge_remappings())
    actuator_lib=str(Path(get_package_prefix('wheeltec_sim_actuator'))/'lib')
    gazebo=ExecuteProcess(cmd=['ign','gazebo','-r','-s',*(['--headless-rendering'] if headless else []),
                               str(sdf)],output='screen',additional_env={
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':actuator_lib+os.pathsep+os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH','')})
    nodes=[gazebo,bridge,
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':robot_urdf(g)}]),
        Node(package='wheeltec_vision_sim',executable='sim_inputs',parameters=[dict(baseline=g.baseline,gyro_noise=g.gyro_noise,accel_noise=g.accel_noise)]),
        Node(package='wheeltec_vision_sim',executable='wheel_odom',parameters=[dict(wheel_radius=g.radius,track=g.track)]),
        Node(package='rtabmap_sync',executable='rgbd_sync',name='vision_rgbd_sync',parameters=[dict(approx_sync=False,qos=1)],
            remappings=[('rgb/image','/vision/paired/infra1/image_rect_raw'),('rgb/camera_info','/vision/paired/infra1/camera_info'),
                        ('depth/image','/camera/depth/image_rect_raw'),('rgbd_image','/rtabmap/rgbd_image')])]
    if estimator=='baseline':
        vo=Node(package='rtabmap_odom',executable='rgbd_odometry',name='vision_rgbd_odometry',output='screen',
            parameters=[package_file('wheeltec_vision_nav','config','rtabmap_visual.yaml'),
            dict(frame_id='base_footprint',odom_frame_id='odom',publish_tf=False,publish_null_when_lost=False,subscribe_rgbd=True,approx_sync=False,qos=1)],
            remappings=[('rgbd_image','/rtabmap/rgbd_image'),('odom','/sim/odom_raw')])
    else:
        config=vins_config(output/'vins',g,estimator)
        vo=Node(package='rtabmap_odom',executable='stereo_odometry',name='sim_vins',output='screen',parameters=[dict(
            frame_id='base_footprint',odom_frame_id='odom',publish_tf=False,publish_null_when_lost=False,approx_sync=False,qos=1,qos_imu=1,
            queue_size=200,imu_queue_size=400,wait_imu_to_init=estimator=='vio',
            **{'Odom/Strategy':'9','OdomVINSFusion/ConfigPath':config,'Rtabmap/ImagesAlreadyRectified':'true','Odom/ResetCountdown':'0'})],
            remappings=[('left/image_rect','/sim/left/image_rect'),('right/image_rect','/sim/right/image_rect'),
                ('left/camera_info','/sim/left/camera_info'),('right/camera_info','/sim/right/camera_info'),
                ('imu','/imu/data_raw' if estimator=='vio' else '/sim/unused_imu'),('odom','/sim/odom_raw')])
    ekf_cfg=yaml.safe_load(Path(package_file('wheeltec_vision_nav','config','ekf_visual.yaml')).read_text())['vision_ekf']['ros__parameters']
    ekf_cfg=ekf_parameters(ekf_cfg,estimator)
    nodes.extend([vo,Node(package='wheeltec_vision_sim',executable='odom_adapter',parameters=[dict(vins=estimator!='baseline',vio=estimator=='vio')]),
        ekf_inputs_node(wheel_prior_used=False,safety=safe),
        Node(package='robot_localization',executable='ekf_node',name='vision_ekf',parameters=[ekf_cfg],remappings=[('odometry/filtered','/odom')]),
        rtabmap_node(str(database),localization=mode=='localization'),
        *depth_obstacle_nodes(safety=safe),
        Node(package='wheeltec_tf_guard',executable='tf_authority_guard',name='vision_tf_authority',parameters=[dict(require_map=mode=='localization',author_timeout_s=safe['map_stale_s'])]),
        vo_watchdog_node(wheel_backup=safe['wheel_backup'],safety=safe)])
    activation=None
    if mode=='localization':
        params=write_visual_nav_params(package_file('wheeltec_vision_nav','config','visual_nav_baseline.yaml'),supervised_reverse=reverse,runtime=cfg)
        nav=simulation_time(yaml.safe_load(Path(params).read_text()))
        sim_params=output/'nav2.yaml';sim_params.write_text(yaml.safe_dump(nav));Path(params).unlink()
        nav_nodes=navigation_nodes(str(sim_params),autostart=False)[:-1]
        nav_nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[dict(
            use_sim_time=True,autostart=False,node_names=['controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother'])]))
        # All Nav2 processes are present for graph readiness, while lifecycle
        # activation is gated on trustworthy map->odom and localization evidence.
        nodes.extend(nav_nodes)
        activation=Node(package='wheeltec_vision_sim',executable='mission_runner',name='sim_navigation_activation',arguments=['activate'])
        nodes.append(activation)
    manifest=dict(mode=mode,estimator=estimator,geometry=g.__dict__,source='synthetic Gazebo Fortress',resource_root_pid=os.getpid(),
        command_chain=['Nav2','/cmd_vel_nav','velocity_smoother','/cmd_vel','vo_watchdog','/cmd_vel_safe','Gazebo Actuator'],
        wall_clock_actuator_timeout_s=.25,simulation_clock_hz=100,simulation_clock_stamp='exact acquired Gazebo nanoseconds; paused stationary, rewind immediate',
        ground_truth='scoring only /sim/ground_truth',vio_independent_imu_fusion=False,safety=safe,
        tf_owners={'map->odom':'RTAB-Map','odom->base_footprint':'EKF','base->sensor':'robot_state_publisher'})
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    # Every failure tears down the whole group; actuator deadline remains last resort.
    leases=acquire_simulation_leases(database)
    handlers=supervised_processes(leases,[n for n in nodes if n is not activation],None,[],[])
    if activation is not None:
        def activated(event,context):
            return [] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='Nav2 activation failed/timed out'))]
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=activation,on_exit=activated)))
    return [SetParameter(name='use_sim_time',value=True),*handlers,*nodes]
