"""Pure simulation parameter transforms; shared with integration launch."""
from copy import deepcopy
import math

def effective_runtime(cfg,g,reverse=False):
    cfg=deepcopy(cfg)
    cfg['safety'].update(footprint_front=max(.01,g.wheelbase/2+g.length/2,g.wheelbase+g.radius),
        footprint_rear=max(.01,g.length/2-g.wheelbase/2,g.radius),
        footprint_half_width=max(g.width/2,g.track/2+.0175),
        min_turning_radius=max(.6,g.track/2+g.wheelbase/math.tan(g.steering_limit)),supervised_reverse=bool(reverse))
    return cfg

def ekf_parameters(cfg,estimator):
    cfg=deepcopy(cfg)
    if estimator=='vio':
        for key in list(cfg):
            if key.startswith('imu0'):cfg.pop(key)
    cfg['use_sim_time']=True
    return cfg

def simulation_time(cfg):
    """Rewrite nested costmap as well as top-level Nav2 time parameters."""
    result=deepcopy(cfg)
    def visit(value):
        if not isinstance(value,dict):return
        if 'ros__parameters' in value:value['ros__parameters']['use_sim_time']=True
        for child in list(value.values()):visit(child)
    visit(result)
    return result
