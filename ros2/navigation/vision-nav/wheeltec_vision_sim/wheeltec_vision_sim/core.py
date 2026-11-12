"""ROS-free encoder integration, synchronized exposure gate and VO contract."""
from dataclasses import dataclass
import math


@dataclass
class EncoderOdometry:
    radius: float=.065
    track: float=.30
    x: float=0.
    y: float=0.
    yaw: float=0.
    last_ns: int | None=None
    previous: tuple | None=None

    def update(self,stamp_ns,left,right):
        if not all(math.isfinite(v) for v in (left,right)) or stamp_ns<=0: return None
        old,last=self.previous,self.last_ns; self.previous=(left,right); self.last_ns=stamp_ns
        if last is None: return None
        dt=(stamp_ns-last)*1e-9
        if not 0<dt<=.5: return None
        dl,dr=(left-old[0])*self.radius,(right-old[1])*self.radius
        if max(abs(dl),abs(dr))>2*dt: return None  # encoder discontinuity, fail closed
        distance=(dl+dr)/2; angle=(dr-dl)/self.track
        if abs(angle)<1e-10: dx,dy=distance,0.
        else: dx,dy=distance*math.sin(angle)/angle,distance*(1-math.cos(angle))/angle
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        self.x+=c*dx-s*dy;self.y+=s*dx+c*dy;self.yaw+=angle
        return self.x,self.y,self.yaw,distance/dt,angle/dt


class ExposureGate:
    """Only release exact integer stamp groups; never alter sensor time evidence."""
    def __init__(self,names,limit=12): self.names=tuple(names); self.pending={}; self.limit=limit
    def add(self,name,stamp,message):
        if name not in self.names or stamp<=0: return None
        self.pending.setdefault(stamp,{})[name]=message
        for key in sorted(self.pending)[:-self.limit]: self.pending.pop(key)
        group=self.pending.get(stamp,{})
        if len(group)!=len(self.names): return None
        self.pending.pop(stamp)
        return group


def body_velocity(previous,current):
    """Pose tuples are (integer ns,x,y,yaw); return child-frame vx,vy,wz."""
    dt=(current[0]-previous[0])*1e-9
    if not .001<=dt<=.5:return None
    dx,dy=(current[1]-previous[1])/dt,(current[2]-previous[2])/dt
    yaw=current[3]
    velocity=(math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,
              math.atan2(math.sin(yaw-previous[3]),math.cos(yaw-previous[3]))/dt)
    if not all(math.isfinite(v) for v in velocity) or math.hypot(*velocity[:2])>2 or abs(velocity[2])>4:return None
    return velocity


def odometry_contract(position,quaternion,velocity,covariance,stamp_ns,now_ns,last_ns=None):
    from wheeltec_vision_nav.runtime_safety import covariance_valid
    if stamp_ns<=0 or (last_ns is not None and stamp_ns<=last_ns): return 'nonmonotonic'
    if not -.05 <= (now_ns-stamp_ns)*1e-9 <= .5: return 'stale_or_future'
    if not all(math.isfinite(v) for v in (*position,*quaternion,*velocity,*covariance)): return 'nonfinite'
    if abs(sum(v*v for v in quaternion)-1)>.02: return 'invalid_orientation'
    if not covariance_valid(covariance) or any(covariance[i]>=1000 for i in (0,7,35)): return 'invalid_covariance'
    return None
