"""Fault assertions consume observations, never infer success from injection."""
import math
SCENARIOS=('motion','route','vision','imu','depth','pause','old_goal','obstacle','actuator_lease')
CONTACT_SOURCES=('body','rear_left','rear_right','front_left','front_right')


class Evidence:
    """Callback receipts/acquisitions and latched events survive recorder sampling."""
    def __init__(self):self.sources={};self.collision_events=0
    def note(self,name,stamp_ns,receipt,collisions=0):
        self.sources[name]=(stamp_ns,receipt)
        self.collision_events+=max(0,collisions)
    def fresh(self,name,now_ns,wall):
        stamp,receipt=self.sources.get(name,(-1,-math.inf))
        return stamp>0 and -.1<=(now_ns-stamp)*1e-9<=.3 and 0<=wall-receipt<=.5
    def snapshot(self,now_ns,wall,paused=False):
        names=('gt','wheel',*CONTACT_SOURCES)
        return dict(evidence_fresh={n:self.fresh(n,now_ns,wall) for n in names},
            collision_events=self.collision_events,pause_expected=paused)


def trial_evidence(samples):
    if not samples:return False
    active=[s for s in samples if not s.get('pause_expected',False)]
    return bool(active) and all(all(s.get('evidence_fresh',{}).get(n,False) for n in ('gt','wheel',*CONTACT_SOURCES)) for s in active)


def physical_stop(samples,max_displacement=.04):
    if len(samples)<2:return False
    if not all(s.get('evidence_fresh',{}).get('gt',False) and s.get('evidence_fresh',{}).get('wheel',False)
               and abs(s.get('wheel_v',math.inf))<=.02 and abs(s.get('wheel_w',math.inf))<=.05 for s in samples):return False
    origin=samples[0].get('ground_truth')
    if origin is None:return False
    xyz=origin['xyz']
    return all(s.get('ground_truth') is not None and math.dist(xyz[:2],s['ground_truth']['xyz'][:2])<=max_displacement for s in samples)

def assess_fault(samples,onset,recovery,deadline=.55,require_latch=False,pause=False,native_lease=False):
    during=[s for s in samples if onset+deadline<=s['wall']<=recovery]
    after=[s for s in samples if recovery+.5<=s['wall']<=recovery+1.5]
    def zero(s):return abs(s['safe_v'])<1e-5 and abs(s['safe_w'])<1e-5 and s.get('safe_age_s',1.) is not None and s.get('safe_age_s',1.)<.3
    active_stop=[s for s in during if not s.get('pause_expected',False) and s['wall']>=onset+max(deadline,.85)]
    resumed=[s for s in samples if recovery+.25<=s['wall']<=recovery+1.5 and not s.get('pause_expected',False)]
    events=[s.get('collision_events',0) for s in samples]
    result=dict(observed=bool(during),safe_zero=bool(during) and all(zero(s) for s in during),
                collisions=max(events,default=0)-min(events,default=0),complete_fresh_evidence=trial_evidence(samples),
                physical_stop=physical_stop(resumed if pause else active_stop))
    if pause:
        paused=[s for s in during if s.get('pause_expected',False)]
        result['paused_clock_stopped']=len(paused)>=2 and max(s['ros_ns'] for s in paused)==min(s['ros_ns'] for s in paused)
    if native_lease:
        # Prove the independent lease caused the measured stop: every sample in
        # that same observation window must still see a fresh nonzero ROS input.
        # A single earlier command followed by watchdog zero/stale output cannot
        # establish the native boundary, even if the vehicle physically stopped.
        ordered=sorted(active_stop,key=lambda s:s['wall'])
        coverage=(len(ordered)>=3 and ordered[-1]['wall']-ordered[0]['wall']>=.5
            and ordered[0]['wall']-(onset+max(deadline,.85))<=.25
            and recovery-ordered[-1]['wall']<=.25
            and all(b['wall']-a['wall']<=.35 for a,b in zip(ordered,ordered[1:])))
        result['native_observation_coverage']=coverage
        result['commands_remained_live']=coverage and all(
            math.isfinite(s.get('safe_v',math.nan)) and abs(s['safe_v'])>.03
            and s.get('safe_age_s') is not None and 0<=s['safe_age_s']<.3 for s in active_stop)
    if require_latch:
        result['old_goal_blocked']=bool(after) and all(zero(s) and s.get('new_goal_required',False) for s in after)
    result['passed']=result['observed'] and (result['safe_zero'] or native_lease) and result['complete_fresh_evidence'] and result['physical_stop'] and not result['collisions'] and result.get('old_goal_blocked',True) and result.get('paused_clock_stopped',True) and result.get('commands_remained_live',True)
    return result


def pose2(truth):
    x,y=truth['xyz'][:2];qx,qy,qz,qw=truth['quaternion']
    return x,y,math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))


def fixed_alignment(map_pose,world_pose):
    angle=world_pose[2]-map_pose[2];c,s=math.cos(angle),math.sin(angle)
    return world_pose[0]-c*map_pose[0]+s*map_pose[1],world_pose[1]-s*map_pose[0]-c*map_pose[1],angle


def world_goal(goal,alignment):
    x,y,a=alignment;c,s=math.cos(a),math.sin(a)
    return x+c*goal[0]-s*goal[1],y+s*goal[0]+c*goal[1],a+goal[2]


def route_result(action_status,goal,alignment,truth,fresh,position_tolerance=.25,yaw_tolerance=.25):
    expected=world_goal(goal,alignment)
    if truth is None:return dict(passed=False,reason='missing_ground_truth')
    actual=pose2(truth);distance=math.dist(expected[:2],actual[:2]);heading=abs(math.atan2(math.sin(expected[2]-actual[2]),math.cos(expected[2]-actual[2])))
    return dict(passed=action_status==4 and fresh and distance<=position_tolerance and heading<=yaw_tolerance,
        action_status=action_status,expected_world_goal=expected,actual_world_endpoint=actual,
        position_error_m=distance,heading_error_rad=heading,position_tolerance_m=position_tolerance,yaw_tolerance_rad=yaw_tolerance)

def resource_snapshot(root_pid=None):
    import os
    from pathlib import Path
    from resource import getrusage,RUSAGE_SELF
    rows=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            pid=int(entry.name)
            text=(entry/'stat').read_text();rest=text[text.rfind(')')+2:].split()
            rows.append(dict(pid=pid,ppid=int(rest[1]),cpu_ticks=int(rest[11])+int(rest[12]),rss_bytes=int(rest[21])*os.sysconf('SC_PAGE_SIZE')))
        except (OSError,ValueError,IndexError):pass
    usage=getrusage(RUSAGE_SELF)
    selected={root_pid or os.getpid()}
    while True:
        descendants={row['pid'] for row in rows if row['ppid'] in selected}
        if descendants<=selected:break
        selected|=descendants
    return dict(processes=[row for row in rows if row['pid'] in selected],resource_root_pid=root_pid or os.getpid(),clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),
        runner_user_s=usage.ru_utime,runner_system_s=usage.ru_stime,runner_max_rss_kib=usage.ru_maxrss)
