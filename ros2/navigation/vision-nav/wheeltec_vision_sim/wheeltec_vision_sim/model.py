"""Deterministic, offline Fortress SDF / TF model generation (metres, radians)."""
from dataclasses import dataclass, asdict
import math
from pathlib import Path
import random
import struct
import zlib
import xml.etree.ElementTree as ET
from .extrinsics import transforms,rpy


@dataclass(frozen=True)
class Geometry:
    wheelbase: float = .34
    track: float = .30
    radius: float = .065
    width: float = .32
    length: float = .46
    height: float = .12
    steering_limit: float = .65
    camera_x: float = .16
    camera_y: float = .0
    camera_z: float = .32
    camera_roll: float = 0.
    camera_pitch: float = 0.
    camera_yaw: float = 0.
    baseline: float = .095
    imu_x: float = 0.
    imu_y: float = 0.
    imu_z: float = .15
    imu_roll: float = 0.
    imu_pitch: float = 0.
    imu_yaw: float = 0.
    width_px: int = 640
    height_px: int = 400
    hfov: float = 1.40
    camera_hz: float = 20.
    imu_hz: float = 200.
    image_noise: float = .002
    gyro_noise: float = .001
    accel_noise: float = .01
    seed: int = 17

    def __post_init__(self):
        values = asdict(self)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('all geometry must be finite')
        if any(values[k] <= 0 for k in ('wheelbase','track','radius','length','width','height',
               'steering_limit','baseline','width_px','height_px','hfov','camera_hz','imu_hz')):
            raise ValueError('positive dimensions required')
        if not .1 < self.hfov < math.pi or self.steering_limit >= math.pi/2:
            raise ValueError('invalid camera/steering angle')
        if min(self.image_noise,self.gyro_noise,self.accel_noise) < 0:
            raise ValueError('negative noise')
        if any(type(values[k]) is not int for k in ('width_px','height_px','seed')):
            raise ValueError('image sizes and random seed must be integers')

    @property
    def fx(self): return self.width_px/(2*math.tan(self.hfov/2))


def _tag(parent,tag_name,text=None,**attributes):
    child=ET.SubElement(parent,tag_name,attributes)
    if text is not None: child.text=str(text)
    return child


def _shape(link, name, size, pose='0 0 0 0 0 0', color='.5 .5 .5 1', collision=True, cylinder=False, texture=None):
    for kind in (('collision','visual') if collision else ('visual',)):
        item=_tag(link,kind,name=name+'_'+kind); _tag(item,'pose',pose)
        geometry=_tag(item,'geometry')
        if cylinder:
            shape=_tag(geometry,'cylinder'); _tag(shape,'radius',size[0]); _tag(shape,'length',size[1])
        else: _tag(_tag(geometry,'box'),'size',' '.join(map(str,size)))
        if kind=='visual':
            material=_tag(item,'material'); _tag(material,'ambient',color); _tag(material,'diffuse',color)
            if texture:
                metal=_tag(_tag(material,'pbr'),'metal')
                _tag(metal,'albedo_map',texture); _tag(metal,'metalness',0); _tag(metal,'roughness',1)
        else:
            ode=_tag(_tag(_tag(item,'surface'),'friction'),'ode'); _tag(ode,'mu',1.0); _tag(ode,'mu2',1.0)


def _inertia(link,mass,size):
    inertial=_tag(link,'inertial'); _tag(inertial,'mass',mass); inertia=_tag(inertial,'inertia')
    x,y,z=size
    for name,value in (('ixx',mass*(y*y+z*z)/12),('iyy',mass*(x*x+z*z)/12),('izz',mass*(x*x+y*y)/12)):
        _tag(inertia,name,value)


def _plugin(parent,filename,name): return _tag(parent,'plugin',filename=filename,name=name)


def _camera(link,g,name,kind,topic,transform):
    sensor=_tag(link,'sensor',name=name,type=kind); _tag(sensor,'pose',' '.join(map(str,(*transform[:3,3],*rpy(transform[:3,:3])))))
    _tag(sensor,'always_on','true'); _tag(sensor,'update_rate',g.camera_hz); _tag(sensor,'topic',topic if kind=='rgbd_camera' else topic+'/image')
    camera=_tag(sensor,'camera'); _tag(camera,'horizontal_fov',g.hfov)
    image=_tag(camera,'image'); _tag(image,'width',g.width_px); _tag(image,'height',g.height_px); _tag(image,'format','R8G8B8')
    clip=_tag(camera,'clip'); _tag(clip,'near',.1); _tag(clip,'far',12.)
    intrinsics=_tag(_tag(camera,'lens'),'intrinsics')
    for key,value in (('fx',g.fx),('fy',g.fx),('cx',(g.width_px-1)/2),('cy',(g.height_px-1)/2),('s',0)):_tag(intrinsics,key,value)
    noise=_tag(camera,'noise',type='gaussian'); _tag(noise,'mean',0); _tag(noise,'stddev',g.image_noise)
    # Fortress RGBDCamera accepts depth_camera clip independently of RGB clip.
    if kind=='rgbd_camera':
        clip=_tag(_tag(camera,'depth_camera'),'clip'); _tag(clip,'near',.15); _tag(clip,'far',6.)


def texture_png(seed, width=2048, height=512):
    """High-contrast nonperiodic corner patterns, generated without image libraries."""
    rng=random.Random(seed)
    pixels=bytearray(bytes((220,220,215))*(width*height))
    def rectangle(x,y,w,h,color):
        row=bytes(color)*w
        for v in range(y,min(y+h,height)):
            start=(v*width+x)*3; pixels[start:start+len(row)]=row
    # Stratification covers every patch of surface. Independent position, size,
    # color and corner notches avoid a repeating checkerboard correspondence.
    for y in range(0,height-40,40):
        for x in range(0,width-40,40):
            left=x+rng.randrange(3,12); top=y+rng.randrange(3,12)
            w,h=rng.randrange(14,26),rng.randrange(14,26)
            color=tuple(rng.randrange(15,70) for _ in range(3))
            rectangle(left,top,w,h,color)
            notch=rng.randrange(4)
            rectangle(left+(w//2 if notch&1 else 0),top+(h//2 if notch&2 else 0),w//2,h//2,(220,220,215))
    def chunk(kind,data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    raw=b''.join(b'\0'+pixels[y*width*3:(y+1)*width*3] for y in range(height))
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b'')


def write_textures(directory,g=Geometry()):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
    for i,name in enumerate(('floor','north','south','east','west')):
        (directory/(str(g.seed)+'-'+name+'.png')).write_bytes(texture_png(g.seed*65537+i,height=2048 if name=='floor' else 512))
    for i in range(5):
        (directory/(str(g.seed)+'-landmark_'+str(i)+'.png')).write_bytes(texture_png(g.seed*65537+5+i,256,256))


def world_sdf(g=Geometry(),texture_directory='textures'):
    root=ET.Element('sdf',version='1.8'); world=_tag(root,'world',name='wheeltec_room')
    physics=_tag(world,'physics',name='1ms',type='ignored'); _tag(physics,'max_step_size',.001); _tag(physics,'real_time_factor',1.)
    for system in ('Physics','UserCommands','SceneBroadcaster','Imu'):
        filename=''.join(('-'+c.lower() if c.isupper() else c) for c in system).lstrip('-')
        _plugin(world,'ignition-gazebo-'+filename+'-system','ignition::gazebo::systems::'+system)
    sensors=_plugin(world,'ignition-gazebo-sensors-system','ignition::gazebo::systems::Sensors'); _tag(sensors,'render_engine','ogre2')
    scene=_tag(world,'scene'); _tag(scene,'ambient','.6 .6 .6 1'); _tag(scene,'background','.15 .18 .2 1')
    light=_tag(world,'light',name='sun',type='directional'); _tag(light,'pose','0 0 8 0 0 0'); _tag(light,'diffuse','.9 .9 .9 1'); _tag(light,'direction','-.3 -.4 -1')
    # Each complete surface has a distinct seeded PNG rather than thousands of
    # individual draw calls. Texture affects rendering only, never collision.
    texture_directory=Path(texture_directory)
    def box(name,xyz,size,color,texture=None):
        model=_tag(world,'model',name=name); _tag(model,'static','true'); _tag(model,'pose',' '.join(map(str,xyz))+' 0 0 0')
        _shape(_tag(model,'link',name='link'),name,size,color=color,texture=texture)
    box('floor',(0,0,-.06),(12,10,.1),'1 1 1 1',str(texture_directory/(str(g.seed)+'-floor.png')))
    for name,xyz,size in [('north',(0,5,1.5),(12,.15,3)),('south',(0,-5,1.5),(12,.15,3)),
                          ('east',(6,0,1.5),(.15,10,3)),('west',(-6,0,1.5),(.15,10,3))]:
        box(name,xyz,size,'1 1 1 1',str(texture_directory/(str(g.seed)+'-'+name+'.png')))
    for i,(x,y) in enumerate(((2,1.8),(0,3.5),(-2,-1.8),(3.8,-2.2),(-4,2))):
        box('landmark_'+str(i),(x,y,.55),(.45,.55,1.1),f'{.1+i*.13} {.8-i*.12} .35 1',str(texture_directory/(str(g.seed)+'-landmark_'+str(i)+'.png')))
    model=_tag(world,'model',name='wheeltec'); _tag(model,'pose','-3 0 0 0 0 0')
    body=_tag(model,'link',name='base_footprint'); _inertia(body,8,(g.length,g.width,g.height))
    _tag(body.find('inertial'),'pose',f'{g.wheelbase/2} 0 {g.radius+g.height/2} 0 0 0')
    _shape(body,'chassis',(g.length,g.width,g.height),pose=f'{g.wheelbase/2} 0 {g.radius+g.height/2} 0 0 0',color='.1 .25 .6 1')
    # Base origin at rear axle: cmd_vel, wheel integration, EKF and swept safety
    # geometry all share the bicycle model's nonholonomic reference point.
    for front,x in ((False,0.),(True,g.wheelbase)):
        for side,y in (('left',g.track/2),('right',-g.track/2)):
            prefix=('front_' if front else 'rear_')+side
            parent='base_footprint'
            if front:
                steer=_tag(model,'link',name='steer_'+side); _tag(steer,'pose',f'{x} {y} {g.radius} 0 0 0'); _inertia(steer,.08,(.025,.025,.025))
                joint=_tag(model,'joint',name='steer_'+side+'_joint',type='revolute'); _tag(joint,'parent',parent); _tag(joint,'child','steer_'+side)
                axis=_tag(joint,'axis'); _tag(axis,'xyz','0 0 1'); limits=_tag(axis,'limit'); _tag(limits,'lower',-g.steering_limit); _tag(limits,'upper',g.steering_limit)
                parent='steer_'+side
            link=_tag(model,'link',name=prefix+'_wheel'); _tag(link,'pose',f'{x} {y} {g.radius} 0 0 0'); _inertia(link,.3,(g.radius*2,.035,g.radius*2))
            _shape(link,prefix,(g.radius,.035),pose='0 0 0 1.57079632679 0 0',color='.06 .06 .06 1',cylinder=True)
            contact=_tag(link,'sensor',name=prefix+'_contact',type='contact');_tag(contact,'topic','/sim/contacts/'+prefix)
            _tag(contact,'always_on','true');_tag(contact,'update_rate',50);_tag(_tag(contact,'contact'),'collision',prefix+'_collision')
            joint=_tag(model,'joint',name=prefix+'_joint',type='revolute'); _tag(joint,'parent',parent); _tag(joint,'child',prefix+'_wheel'); _tag(_tag(joint,'axis'),'xyz','0 1 0')
    calibration=transforms(g)
    _camera(body,g,'left_rgbd','rgbd_camera','/sim/raw/left',calibration['left_physical'])
    _camera(body,g,'right_rgb','camera','/sim/raw/right',calibration['right_physical'])
    imu=_tag(body,'sensor',name='imu',type='imu'); _tag(imu,'pose',' '.join(map(str,(*calibration['imu'][:3,3],g.imu_roll,g.imu_pitch,g.imu_yaw))))
    _tag(imu,'topic','/sim/raw/imu'); _tag(imu,'always_on','true'); _tag(imu,'update_rate',g.imu_hz)
    detail=_tag(imu,'imu')
    for group,noise in (('angular_velocity',g.gyro_noise),('linear_acceleration',g.accel_noise)):
        element=_tag(detail,group)
        for axis in ('x','y','z'):
            n=_tag(_tag(element,axis),'noise',type='gaussian'); _tag(n,'mean',0); _tag(n,'stddev',noise)
    contact=_tag(body,'sensor',name='body_contact',type='contact'); _tag(contact,'topic','/sim/contacts'); _tag(contact,'always_on','true'); _tag(contact,'update_rate',50); _tag(_tag(contact,'contact'),'collision','chassis_collision')
    actuator=_plugin(model,'libwheeltec_sim_actuator.so','wheeltec::Actuator')
    for name,value in (('seed',g.seed),('wheelbase',g.wheelbase),('track',g.track),('radius',g.radius),('timeout',.25),
                       ('steering_limit',g.steering_limit),('min_radius',max(.6,g.track/2+g.wheelbase/math.tan(g.steering_limit)))): _tag(actuator,name,value)
    # Native actuator publishes measured joint states at 50 Hz. Older installed
    # Fortress stock JointStatePublisher ignores update_rate and floods at 1 kHz.
    truth=_plugin(model,'ignition-gazebo-odometry-publisher-system','ignition::gazebo::systems::OdometryPublisher')
    for name,value in (('odom_topic','/sim/ground_truth'),('odom_publish_frequency',50),('odom_frame','world'),('robot_base_frame','ground_truth/base_footprint'),('dimensions',3)): _tag(truth,name,value)
    return ET.tostring(root,encoding='unicode')


def robot_urdf(g=Geometry()):
    root=ET.Element('robot',name='wheeltec_sim');body=_tag(root,'link',name='base_footprint')
    visual=_tag(body,'visual');_tag(visual,'origin',xyz=f'{g.wheelbase/2} 0 {g.radius+g.height/2}',rpy='0 0 0')
    _tag(_tag(visual,'geometry'),'box',size=f'{g.length} {g.width} {g.height}')
    _tag(_tag(visual,'material',name='blue'),'color',rgba='.1 .25 .6 1')
    for front,x in ((False,0.),(True,g.wheelbase)):
        for side,y in (('left',g.track/2),('right',-g.track/2)):
            prefix=('front_' if front else 'rear_')+side;parent='base_footprint'
            if front:
                _tag(root,'link',name='steer_'+side);steer=_tag(root,'joint',name='steer_'+side+'_joint',type='revolute')
                _tag(steer,'parent',link=parent);_tag(steer,'child',link='steer_'+side)
                _tag(steer,'origin',xyz=f'{x} {y} {g.radius}',rpy='0 0 0');_tag(steer,'axis',xyz='0 0 1')
                _tag(steer,'limit',lower=str(-g.steering_limit),upper=str(g.steering_limit),effort='20',velocity='2')
                parent='steer_'+side
            wheel=_tag(root,'link',name=prefix+'_wheel');visual=_tag(wheel,'visual')
            _tag(visual,'origin',xyz='0 0 0',rpy='1.5707963267948966 0 0')
            _tag(_tag(visual,'geometry'),'cylinder',radius=str(g.radius),length='.035')
            _tag(_tag(visual,'material',name='rubber'),'color',rgba='.06 .06 .06 1')
            joint=_tag(root,'joint',name=prefix+'_joint',type='continuous');_tag(joint,'parent',link=parent);_tag(joint,'child',link=prefix+'_wheel')
            _tag(joint,'origin',xyz='0 0 0' if front else f'{x} {y} {g.radius}',rpy='0 0 0');_tag(joint,'axis',xyz='0 1 0')
    calibration=transforms(g)
    for name in ('left','right','imu'):
        tf=calibration[name]
        child='sim_'+name+('_optical' if name!='imu' else '')
        _tag(root,'link',name=child); joint=_tag(root,'joint',name=child+'_fixed',type='fixed')
        _tag(joint,'parent',link='base_footprint'); _tag(joint,'child',link=child)
        _tag(joint,'origin',xyz=' '.join(map(str,tf[:3,3])),rpy=' '.join(map(str,rpy(tf[:3,:3]))))
    return ET.tostring(root,encoding='unicode')


def write_model(directory,g=Geometry()):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
    sdf=directory/'room.sdf'; urdf=directory/'robot.urdf'
    write_textures(directory/'textures',g)
    sdf.write_text(world_sdf(g,(directory/'textures').resolve()),encoding='utf-8'); urdf.write_text(robot_urdf(g),encoding='utf-8')
    return sdf,urdf
