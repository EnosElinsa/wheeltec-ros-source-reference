import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from wheeltec_vision_sim.core import EncoderOdometry,ExposureGate,odometry_contract,body_velocity
from wheeltec_vision_sim.model import Geometry,world_sdf,robot_urdf
from wheeltec_vision_sim.extrinsics import transforms,rotation_rpy
from wheeltec_vision_sim.mission_logic import assess_fault,CONTACT_SOURCES
from wheeltec_vision_sim.configuration import effective_runtime,ekf_parameters,simulation_time


def test_encoder_integrates_measured_arc_and_reverse_without_ground_truth():
    odom=EncoderOdometry(radius=.1,track=.4)
    assert odom.update(1_000_000_000,0.,0.) is None
    # Rear wheel travel .08/.12 metres -> rear centre travels .1m, yaw .1rad.
    pose=odom.update(1_500_000_000,.8,1.2)
    assert pose==pytest.approx((math.sin(.1),1-math.cos(.1),.1,.2,.2))
    pose=odom.update(2_000_000_000,0.,0.)
    assert pose[:3]==pytest.approx((0,0,0),abs=1e-12)


@pytest.mark.parametrize('stamp,left,right',[(1_000_000_000,.1,.1),(900_000_000,.1,.1),(2_000_000_000,1.,1.),(1_100_000_000,100.,1.),(1_100_000_000,float('nan'),1.)])
def test_encoder_rejects_duplicates_jumps_and_invalid_values(stamp,left,right):
    odom=EncoderOdometry();odom.update(1_000_000_000,0,0)
    assert odom.update(stamp,left,right) is None
    assert odom.x==odom.y==odom.yaw==0


def test_exact_sync_does_not_pair_adjacent_exposures_and_is_bounded():
    gate=ExposureGate(('image','info'),limit=3)
    assert gate.add('image',1,'old') is None
    assert gate.add('info',2,'new') is None
    assert gate.add('image',2,'same')=={'image':'same','info':'new'}
    for stamp in range(3,30):gate.add('image',stamp,stamp)
    assert len(gate.pending)==3
    assert gate.add('info',29,'info')=={'image':29,'info':'info'}


def test_model_has_real_joints_distinct_gt_and_deterministic_texture():
    text=world_sdf();root=ET.fromstring(text)
    assert text==world_sdf()
    assert text!=world_sdf(Geometry(seed=18))
    model=root.find("world/model[@name='wheeltec']")
    assert len(model.findall('joint'))==6
    assert len(root.findall("world/model[@name='seeded_texture']/link/visual"))==360
    assert len(model.findall("link[@name='base_footprint']/sensor"))==4
    plugins={p.attrib['name']:p for p in model.findall('plugin')}
    assert plugins['wheeltec::Actuator'].findtext('timeout')=='.25' or float(plugins['wheeltec::Actuator'].findtext('timeout'))==.25
    assert plugins['ignition::gazebo::systems::OdometryPublisher'].findtext('odom_topic')=='/sim/ground_truth'
    assert '/cmd_vel' not in text  # the native actuator hardcodes its safe-only transport input


def test_stereo_extrinsics_and_tf_are_same_optical_geometry():
    g=Geometry(camera_yaw=.3,camera_pitch=.1,imu_yaw=-.2)
    tf=transforms(g);left_to_right=np.linalg.inv(tf['left'])@tf['right']
    assert left_to_right[:3,:3]==pytest.approx(np.eye(3))
    assert left_to_right[:3,3]==pytest.approx((g.baseline,0,0))
    assert tf['left'][:3,:3]@np.array([0.,0.,1.])==pytest.approx(rotation_rpy(0,.1,.3)@np.array([1.,0.,0.]))
    root=ET.fromstring(robot_urdf(g))
    joint=root.find("joint[@name='sim_left_optical_fixed']/origin")
    rotation=rotation_rpy(*map(float,joint.attrib['rpy'].split()))
    assert rotation==pytest.approx(tf['left'][:3,:3])


@pytest.mark.parametrize('kwargs',[{'radius':0},{'track':float('nan')},{'steering_limit':2},{'image_noise':-1},{'hfov':4}])
def test_geometry_rejects_invalid_dimensions(kwargs):
    with pytest.raises(ValueError):Geometry(**kwargs)


def test_odom_contract_rejects_stale_wrong_orientation_and_covariance():
    args=[(0,0,0),(0,0,0,1),(1,0,0,0,0,0),[0.]*36,1_000_000_000,1_100_000_000]
    assert odometry_contract(*args) is None
    assert odometry_contract(*args,last_ns=args[4])=='nonmonotonic'
    args[5]=2_000_000_000;assert odometry_contract(*args)=='stale_or_future'
    args[5]=1_000_000_000;args[1]=(0,0,0,0);assert odometry_contract(*args)=='invalid_orientation'
    args[1]=(0,0,0,1);args[3][35]=9999.;assert odometry_contract(*args)=='invalid_covariance'


def test_fault_assertion_requires_real_fresh_zero_evidence_and_latch():
    assert not assess_fault([],0,2)['passed']
    sample=dict(safe_v=0.,safe_w=0.,safe_age_s=.05,new_goal_required=True,contacts=0,contacts_observed=True,
                evidence_fresh={n:True for n in ('gt','wheel',*CONTACT_SOURCES)},wheel_v=0.,wheel_w=0.,
                collision_events=0,ground_truth={'xyz':[0,0,0]})
    samples=[dict(sample,wall=t) for t in (.8,1.,1.5,2.6,3.)]
    assert assess_fault(samples,0,2,require_latch=True)['passed']
    samples[-1]['new_goal_required']=False
    assert not assess_fault(samples,0,2,require_latch=True)['passed']
    samples[-1]['new_goal_required']=True;samples[1]['safe_age_s']=1.
    assert not assess_fault(samples,0,2)['passed']


def test_model_camera_topics_use_each_fortress_sensor_contract():
    root=ET.fromstring(world_sdf());body=root.find("world/model[@name='wheeltec']/link[@name='base_footprint']")
    assert body.find("sensor[@name='left_rgbd']/topic").text=='/sim/raw/left'
    assert body.find("sensor[@name='right_rgb']/topic").text=='/sim/raw/right/image'
    for name in ('left_rgbd','right_rgb'):
        camera=body.find(f"sensor[@name='{name}']/camera")
        assert float(camera.findtext('lens/intrinsics/fx'))==pytest.approx(Geometry().fx)


def test_geometry_runtime_footprint_covers_all_collision_geometry():
    from wheeltec_vision_nav.nav_params import bundled_runtime_config,build_visual_nav_params
    from pathlib import Path
    import yaml
    source=bundled_runtime_config();g=Geometry(wheelbase=.5,length=.65,track=.45,radius=.09)
    cfg=effective_runtime(source,g,True);safe=cfg['safety']
    assert safe['footprint_front']>=g.wheelbase+g.radius
    assert safe['footprint_rear']>=g.radius
    assert safe['footprint_half_width']>=g.track/2+.0175
    assert safe['min_turning_radius']>=g.track/2+g.wheelbase/math.tan(g.steering_limit)
    template=Path(__file__).resolve().parents[2]/'wheeltec_vision_nav/config/visual_nav_baseline.yaml'
    nav=build_visual_nav_params(yaml.safe_load(template.read_text()),'test.xml',supervised_reverse=True,runtime=cfg)
    assert nav['controller_server']['ros__parameters']['FollowPath']['AckermannConstraints']['min_turning_r']==safe['min_turning_radius']
    assert nav['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']==safe['min_turning_radius']
    assert nav['controller_server']['ros__parameters']['FollowPath']['vx_min']==-.08
    assert source['safety']['footprint_front']==.25  # physical baseline remains untouched


def test_vio_never_double_fuses_imu_and_all_nested_costmaps_use_simtime():
    cfg=dict(imu0='/vision/imu',imu0_config=[True],imu0_queue_size=20,odom0='/wheel/odom',odom1='/vision/odom_vo')
    assert not any(k.startswith('imu0') for k in ekf_parameters(cfg,'vio'))
    assert ekf_parameters(cfg,'baseline')['imu0']=='/vision/imu'
    nav={'local_costmap':{'local_costmap':{'ros__parameters':{'use_sim_time':False}}},'controller':{'ros__parameters':{}}}
    updated=simulation_time(nav)
    assert updated['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] is True
    assert updated['controller']['ros__parameters']['use_sim_time'] is True
    assert nav['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] is False


def test_vins_pose_difference_returns_body_velocity_with_wrap_safe_heading():
    assert body_velocity((1_000_000_000,0.,0.,math.pi/2),(1_100_000_000,0.,.02,math.pi/2))==pytest.approx((.2,0.,0.))
    assert body_velocity((1_000_000_000,0.,0.,math.pi-.01),(1_100_000_000,0.,0.,-math.pi+.01))==pytest.approx((0.,0.,.2))
    assert body_velocity((1_000_000_000,0.,0.,0.),(1_100_000_000,2.,0.,0.)) is None
    assert body_velocity((1_000_000_000,0.,0.,0.),(2_000_000_000,.1,0.,0.)) is None


def test_urdf_and_sdf_joint_origins_match_physical_vehicle():
    sdf=ET.fromstring(world_sdf()).find("world/model[@name='wheeltec']")
    urdf=ET.fromstring(robot_urdf())
    assert {j.attrib['name'] for j in sdf.findall('joint')} <= {j.attrib['name'] for j in urdf.findall('joint')}
    rear=list(map(float,urdf.find("joint[@name='rear_left_joint']/origin").attrib['xyz'].split()))
    front=list(map(float,urdf.find("joint[@name='steer_left_joint']/origin").attrib['xyz'].split()))
    assert front[0]-rear[0]==pytest.approx(Geometry().wheelbase)
    assert front[1]==rear[1]==Geometry().track/2
