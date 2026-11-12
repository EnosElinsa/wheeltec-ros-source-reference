"""Live mission/fault experiment with wall-clock evidence; no inferred passes."""
import argparse,json,math,random,subprocess,time
from pathlib import Path

def ign_service(service,request_type,response_type,request):
    result=subprocess.run(['ign','service','-s',service,'--reqtype',request_type,'--reptype',response_type,
        '--timeout','5000','--req',request],capture_output=True,text=True,timeout=10)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError(f'Gazebo service {service}: {result.stdout} {result.stderr}')

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image,Imu
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ManageLifecycleNodes
    from ros_gz_interfaces.msg import Contacts
    from std_msgs.msg import Bool,String
    from std_srvs.srv import SetBool,Trigger
    from tf2_ros import Buffer,TransformListener
    from rclpy.time import Time
    from .mission_logic import (SCENARIOS,Evidence,CONTACT_SOURCES,assess_fault,resource_snapshot,
                               trial_evidence,physical_stop,fixed_alignment,pose2,route_result)
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('activate');run=sub.add_parser('run');run.add_argument('--output',required=True)
    run.add_argument('--mode',choices=['mapping','localization'],default='mapping')
    run.add_argument('--repeats',type=int,default=3);run.add_argument('--seed',type=int,default=17)
    run.add_argument('--scenario',choices=['all',*SCENARIOS],default='all')
    run.add_argument('--estimator',choices=['baseline','stereo','vio'],default='baseline')
    run.add_argument('--launch-manifest',help='generated launch manifest; enables full simulator process-tree CPU/RSS metrics')
    args,rosargs=parser.parse_known_args()
    if args.command=='run' and args.repeats<3:parser.error('each scenario requires at least three repeats')
    root_pid=json.loads(Path(args.launch_manifest).read_text()).get('resource_root_pid') if args.command=='run' and args.launch_manifest else None
    rclpy.init(args=rosargs)

    class Runner(Node):
        def __init__(self):
            super().__init__('sim_mission_runner')
            self.ready=False;self.status={};self.safe=(0.,0.);self.wheel=(0.,0.)
            self.gt=None;self.odom=None;self.contacts=0;self.contacts_seen=False;self.contact_sources={};self.stamps={};self.last_safe=-math.inf
            self.samples=[];self.file=None;self.phase='initializing';self.last_record=0.;self.last_resource=0.
            self.resources={};self.clock_last=None;self.clock_ratio=None
            self.evidence=Evidence();self.paused=False;self.resume_grace_until=0.;self.goal_map=None
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Bool,'/vision/ready',lambda m:setattr(self,'ready',m.data),qos)
            self.create_subscription(String,'/vision/health/status',self.health,10)
            self.create_subscription(Twist,'/cmd_vel_safe',self.command,10)
            self.create_subscription(Odometry,'/wheel/odom',self.encoder,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
            self.create_subscription(Odometry,'/sim/ground_truth',self.truth,qos_profile_sensor_data)
            for part in CONTACT_SOURCES:
                topic='/sim/contacts'+('' if part=='body' else '/'+part)
                self.create_subscription(Contacts,topic,lambda m,n=part:self.contact(n,m),qos_profile_sensor_data)
            for name,kind,topic in [('vision',Image,'/sim/left/image_rect'),('depth',Image,'/camera/depth/image_rect_raw'),('imu',Imu,'/imu/data_raw')]:
                self.create_subscription(kind,topic,lambda m,n=name:self.note(n,m),qos_profile_sensor_data)
            self.pub=self.create_publisher(Twist,'/cmd_vel',1)
            self.drop_pub=self.create_publisher(Bool,'/sim/actuator/drop_commands',1)
            self.action=ActionClient(self,NavigateToPose,'navigate_to_pose')
            self.clients={n:self.create_client(SetBool,'/sim/fault/'+n) for n in ('vision','depth','imu')}
            self.rearm=self.create_client(Trigger,'/vision/rearm')
            self.manager=self.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
            self.goal=None
        def note(self,name,msg):
            self.stamps[name]=dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,receipt_wall=time.monotonic())
        def health(self,msg):
            try:self.status=json.loads(msg.data)
            except ValueError:pass
        def command(self,msg):self.safe=(msg.linear.x,msg.angular.z);self.last_safe=time.monotonic()
        def encoder(self,msg):
            self.wheel=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
            self.evidence.note('wheel',msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,time.monotonic())
        def truth(self,msg):
            self.gt=msg;self.evidence.note('gt',msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,time.monotonic())
        def contact(self,name,msg):
            def floor(collision):return collision.name.startswith('floor::') or collision.name=='floor_collision'
            self.contact_sources[name]=sum(not floor(c.collision1) and not floor(c.collision2) for c in msg.contacts)
            self.contacts=sum(self.contact_sources.values());self.contacts_seen=bool(self.contact_sources)
            self.evidence.note(name,msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,time.monotonic(),self.contact_sources[name])
        def spin(self,duration,v=0.,w=0.,drive=False):
            until=time.monotonic()+duration
            while rclpy.ok() and time.monotonic()<until:
                if drive:
                    cmd=Twist();cmd.linear.x=v;cmd.angular.z=w;self.pub.publish(cmd)
                rclpy.spin_once(self,timeout_sec=.025);self.record()
        def record(self):
            wall=time.monotonic()
            if not self.file or wall-self.last_record<.1:return
            self.last_record=wall;ros=self.get_clock().now().nanoseconds
            if self.clock_last:self.clock_ratio=(ros-self.clock_last[1])*1e-9/(wall-self.clock_last[0])
            self.clock_last=(wall,ros)
            if wall-self.last_resource>1:self.last_resource=wall;self.resources=resource_snapshot(root_pid)
            age=wall-self.last_safe
            sample=dict(wall=wall,ros_ns=ros,phase=self.phase,safe_v=self.safe[0],safe_w=self.safe[1],
                safe_age_s=age if math.isfinite(age) else None,wheel_v=self.wheel[0],wheel_w=self.wheel[1],
                new_goal_required=self.status.get('new_goal_required',False),health=self.status,
                ready=self.ready,contacts=self.contacts,contacts_observed=self.contacts_seen,ground_truth=self.pose(self.gt),odom=self.pose(self.odom),
                sensors=self.stamps.copy(),resources=self.resources,sim_per_wall=self.clock_ratio)
            sample.update(self.evidence.snapshot(ros,wall,self.paused or wall<self.resume_grace_until))
            self.samples.append(sample);self.file.write(json.dumps(sample,allow_nan=False)+'\n');self.file.flush()
        def wait(self,predicate,timeout,label):
            until=time.monotonic()+timeout
            while not predicate() and time.monotonic()<until:self.spin(.05)
            if not predicate():raise RuntimeError('timeout waiting for '+label)
        @staticmethod
        def pose(msg):
            if msg is None:return None
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            return dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,frame=msg.header.frame_id,xyz=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w])
        def initial_alignment(self):
            self.wait(lambda:self.evidence.fresh('gt',self.get_clock().now().nanoseconds,time.monotonic()) and abs(self.wheel[0])<.02,15,'fresh stationary initial ground truth')
            tf=self.tf.lookup_transform('map','base_footprint',Time.from_msg(self.gt.header.stamp))
            p,q=tf.transform.translation,tf.transform.rotation
            map_pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            return fixed_alignment(map_pose,pose2(self.pose(self.gt)))
        def call(self,client,request):
            self.wait(client.service_is_ready,15,'service '+client.srv_name)
            future=client.call_async(request);self.wait(future.done,15,'service reply');result=future.result()
            if hasattr(result,'success') and not result.success:raise RuntimeError(str(result))
            return result
        def fault(self,name,value):
            req=SetBool.Request();req.data=value;return self.call(self.clients[name],req)
        def send_goal(self,distance=.7,turn=0.):
            self.wait(self.action.server_is_ready,20,'NavigateToPose')
            tf=self.tf.lookup_transform('map','base_footprint',Time());p,q=tf.transform.translation,tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
            goal.pose.pose.position.x=p.x+distance*math.cos(yaw);goal.pose.pose.position.y=p.y+distance*math.sin(yaw)
            goal.pose.pose.orientation.z=math.sin((yaw+turn)/2);goal.pose.pose.orientation.w=math.cos((yaw+turn)/2)
            self.goal_map=(goal.pose.pose.position.x,goal.pose.pose.position.y,yaw+turn)
            future=self.action.send_goal_async(goal);self.wait(future.done,10,'goal acceptance');self.goal=future.result()
            if not self.goal.accepted:raise RuntimeError('navigation goal rejected')
            return self.goal.get_result_async()
        def recover(self):
            self.wait(lambda:self.ready,45,'healthy recovery')
            if args.mode=='mapping':self.call(self.rearm,Trigger.Request())
        def close_goal(self):
            if self.goal:
                future=self.goal.cancel_goal_async();self.wait(future.done,10,'goal cancellation');self.goal=None
        def moving(self):
            until=time.monotonic()+20
            while time.monotonic()<until:
                self.spin(.1,.1,0.,args.mode=='mapping')
                if abs(self.safe[0])>.03 and abs(self.wheel[0])>.02:return
            raise RuntimeError('no observed nonzero safe command and encoder motion before fault')
    node=Runner()
    try:
        if args.command=='activate':
            node.wait(lambda:node.ready,150,'RTAB-Map localization + map->odom + safety readiness')
            request=ManageLifecycleNodes.Request();request.command=ManageLifecycleNodes.Request.STARTUP
            node.call(node.manager,request);node.get_logger().info('Nav2 activated after localization safety readiness')
            return
        out=Path(args.output).expanduser().resolve();out.mkdir(parents=True,exist_ok=False)
        node.file=(out/'samples.jsonl').open('x',encoding='utf-8');rng=random.Random(args.seed);results=[]
        node.phase='startup';node.wait(lambda:node.ready,150,'startup readiness')
        node.wait(lambda:all(node.evidence.fresh(n,node.get_clock().now().nanoseconds,time.monotonic()) for n in ('gt','wheel',*CONTACT_SOURCES)),20,'all five contact streams and ground truth')
        alignment=node.initial_alignment() if args.mode=='localization' else None
        scenarios=tuple(s for s in SCENARIOS if s!=('route' if args.mode=='mapping' else 'motion')) if args.scenario=='all' else (args.scenario,)
        for scenario in scenarios:
            for trial in range(args.repeats):
                record=dict(scenario=scenario,trial=trial,seed=args.seed,started_wall=time.monotonic())
                start_index=len(node.samples);collision_start=node.evidence.collision_events;node.phase=f'{scenario}/{trial}'
                try:
                    node.recover()
                    if scenario=='motion':
                        if args.mode=='localization':raise RuntimeError('motion primitives require mapping; route uses Nav2')
                        for name,v,w,duration in [('straight',.12,0.,3.),('turn',.1,.12,4.),('stop',0.,0.,1.)]:
                            node.phase=f'{scenario}/{trial}/{name}';node.spin(duration,v,w,True)
                        observed=node.samples[start_index:]
                        record.update(passed=any(abs(s['wheel_v'])>.04 for s in observed) and any(abs(s['wheel_w'])>.03 for s in observed) and abs(node.wheel[0])<.02)
                    elif scenario=='route':
                        if args.mode!='localization':raise RuntimeError('route requires localization launch with saved map')
                        result=node.send_goal(.65,.1);node.wait(result.done,90,'route arrival')
                        record.update(route_result(result.result().status,node.goal_map,alignment,node.pose(node.gt),
                            node.evidence.fresh('gt',node.get_clock().now().nanoseconds,time.monotonic())),requested_map_goal=node.goal_map,fixed_map_to_world=alignment)
                    elif scenario=='obstacle':
                        if not node.evidence.fresh('gt',node.get_clock().now().nanoseconds,time.monotonic()):raise RuntimeError('fresh independent Gazebo ground truth unavailable')
                        p,q=node.gt.pose.pose.position,node.gt.pose.pose.orientation
                        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));name=f'fault_obstacle_{trial}_{rng.randrange(100000)}'
                        sdf=f'<sdf version="1.8"><model name="{name}"><static>true</static><pose>{p.x+.85*math.cos(yaw)} {p.y+.85*math.sin(yaw)} .5 0 0 {yaw}</pose><link name="body"><collision name="box"><geometry><box><size>.2 .8 1</size></box></geometry></collision><visual name="box"><geometry><box><size>.2 .8 1</size></box></geometry></visual></link></model></sdf>'
                        sdf_path=out/(name+'.sdf');sdf_path.write_text(sdf)
                        ign_service('/world/wheeltec_room/create','ignition.msgs.EntityFactory','ignition.msgs.Boolean',f'sdf_filename: "{sdf_path.as_posix()}"')
                        if args.mode=='mapping':node.spin(5,.15,0.,True)
                        else:node.send_goal(1.2);node.spin(5)
                        record.update(passed=node.contacts_seen and abs(node.safe[0])<1e-5 and not any(s['contacts'] for s in node.samples[start_index:]))
                        ign_service('/world/wheeltec_room/remove','ignition.msgs.Entity','ignition.msgs.Boolean',f'name: "{name}", type: MODEL')
                    else:
                        if args.mode=='localization':node.send_goal(1.2)
                        node.moving();record['pre_fault_motion_observed']=True
                        fault='vision' if scenario=='old_goal' else scenario;onset=time.monotonic()
                        if fault=='pause':
                            node.paused=True;ign_service('/world/wheeltec_room/control','ignition.msgs.WorldControl','ignition.msgs.Boolean','pause: true')
                        elif fault=='actuator_lease':node.drop_pub.publish(Bool(data=True));node.spin(.1)
                        else:node.fault(fault,True)
                        node.spin(2.,.1,0.,args.mode=='mapping');recovery=time.monotonic()
                        if fault=='pause':
                            ign_service('/world/wheeltec_room/control','ignition.msgs.WorldControl','ignition.msgs.Boolean','pause: false');node.paused=False;node.resume_grace_until=time.monotonic()+.3
                        elif fault=='actuator_lease':
                            node.close_goal();node.spin(.3,drive=args.mode=='mapping');node.drop_pub.publish(Bool(data=False))
                        else:node.fault(fault,False)
                        node.spin(2.,.1 if fault!='actuator_lease' else 0.,0.,args.mode=='mapping')
                        deadline=1.6 if fault=='vision' and args.estimator!='vio' else .55
                        record.update(assess_fault(node.samples[start_index:],onset,recovery,deadline,
                            require_latch=(scenario=='old_goal' or args.mode=='localization') and fault!='actuator_lease',
                            pause=fault=='pause',native_lease=fault=='actuator_lease'),onset_wall=onset,recovery_wall=recovery)
                    node.close_goal()
                except Exception as error:record.update(passed=False,error=f'{type(error).__name__}: {error}')
                finally:
                    if args.mode=='mapping':node.spin(.3,drive=True)
                observed=node.samples[start_index:]
                record['complete_fresh_evidence']=trial_evidence(observed)
                record['collision_events']=node.evidence.collision_events-collision_start
                record['passed']=record.get('passed',False) and record['complete_fresh_evidence'] and record['collision_events']==0
                record['finished_wall']=time.monotonic();results.append(record)
                (out/'results.json').write_text(json.dumps(dict(schema=1,seed=args.seed,repeats=args.repeats,
                    mode=args.mode,estimator=args.estimator,results=results),indent=2)+'\n')
        if not all(r['passed'] for r in results):raise SystemExit(2)
    finally:
        if node.file:node.file.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
