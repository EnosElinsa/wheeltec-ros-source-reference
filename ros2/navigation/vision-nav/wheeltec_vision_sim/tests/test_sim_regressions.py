import math
from types import SimpleNamespace
import numpy as np
import pytest
from wheeltec_vision_sim.depth import millimetres
from wheeltec_vision_sim.core import EncoderOdometry
from wheeltec_vision_sim.mission_logic import (Evidence,CONTACT_SOURCES,assess_fault,physical_stop,
    trial_evidence,fixed_alignment,route_result)


@pytest.mark.parametrize('bigendian',[True,False])
def test_fortress_metre_depth_converts_endian_padding_without_invalid_fabrication(bigendian):
    values=np.array([[.2,1.234,3.,np.nan,99.],[np.inf,-np.inf,0.,-1.,99.]],dtype='>f4' if bigendian else '<f4')
    out=millimetres('32FC1',4,2,20,bigendian,values.tobytes())
    assert np.frombuffer(out,dtype='<u2').reshape(2,4).tolist()==[[200,1234,3000,0],[0,0,0,0]]


def test_adapted_realistic_depth_passes_existing_gate_and_empty_invalid_does_not():
    from wheeltec_vision_nav.depth_validity import depth_has_valid_returns
    raw=np.full((20,20),1.5,dtype='<f4')
    image=SimpleNamespace(width=20,height=20,step=40,encoding='16UC1',is_bigendian=False,
                          data=millimetres('32FC1',20,20,80,False,raw.tobytes()))
    assert depth_has_valid_returns(image)
    raw[:]=np.nan;image.data=millimetres('32FC1',20,20,80,False,raw.tobytes())
    assert not depth_has_valid_returns(image)


@pytest.mark.parametrize('args',[('16UC1',1,1,2,False,b'00'),('32FC1',2,1,4,False,b'0000'),('32FC1',1,2,4,False,b'0000')])
def test_depth_malformed_or_wrong_units_fail_closed(args):
    with pytest.raises(ValueError):millimetres(*args)


def test_rejected_duplicate_or_backward_never_changes_next_valid_encoder_delta():
    for bad_stamp in (1_000_000_000,900_000_000):
        odom=EncoderOdometry();odom.update(1_000_000_000,0,0)
        assert odom.update(bad_stamp,1,1) is None
        result=odom.update(1_100_000_000,.1,.1)
        assert result==pytest.approx((.0065,0,0,.065,0))
        assert odom.resets==0


def test_discontinuity_resets_epoch_explicitly_without_integrating_jump():
    odom=EncoderOdometry();odom.update(1_000_000_000,0,0)
    assert odom.update(1_100_000_000,100,100) is None
    assert odom.resets==1 and odom.x==0
    assert odom.update(1_200_000_000,100.1,100.1)[0]==pytest.approx(.0065)


def samples():
    return [dict(wall=t,safe_v=0.,safe_w=0.,safe_age_s=.05,wheel_v=0.,wheel_w=0.,
                 evidence_fresh={n:True for n in ('gt','wheel',*CONTACT_SOURCES)},
                 ground_truth={'xyz':[0.,0.,0.]},collision_events=0,new_goal_required=True)
            for t in (1.,1.2,1.5,1.8,2.6,2.9,3.2)]


def test_zero_commands_but_wheels_or_ground_truth_still_move_never_pass():
    data=samples();assert assess_fault(data,0,2,require_latch=True)['passed']
    data[2]['wheel_v']=.1
    assert not assess_fault(data,0,2)['passed']
    data=samples();data[2]['ground_truth']['xyz'][0]=.08
    assert not assess_fault(data,0,2)['passed']


def test_all_five_contacts_and_gt_must_be_fresh_and_collision_event_is_latched():
    evidence=Evidence()
    for name in ('gt','wheel',*CONTACT_SOURCES):evidence.note(name,1_000_000_000,10.)
    assert all(evidence.snapshot(1_100_000_000,10.1)['evidence_fresh'].values())
    assert not any(evidence.snapshot(1_100_000_000,11.)['evidence_fresh'].values())
    evidence.note('body',1_100_000_000,10.1,collisions=1)
    evidence.note('body',1_110_000_000,10.11,collisions=0)
    assert evidence.collision_events==1  # empty callback cannot erase a hit before 10Hz sampling
    for name in ('gt',*CONTACT_SOURCES):
        data=samples();data[2]['evidence_fresh'][name]=False
        assert not trial_evidence(data)
        assert not assess_fault(data,0,2)['passed']


def test_pause_requires_resumed_physics_stop_not_frozen_nonzero_encoder_cache():
    data=samples()
    for sample in data[:4]:sample.update(pause_expected=True,ros_ns=1_000_000_000,wheel_v=.1)
    assert assess_fault(data,0,2,pause=True)['passed']
    data[-1]['ground_truth']['xyz'][0]=.1
    assert not assess_fault(data,0,2,pause=True)['passed']


def test_native_expiry_requires_physical_stop_while_safe_commands_remain_nonzero():
    data=samples()
    for sample in data:sample['safe_v']=.1
    assert assess_fault(data,0,2,native_lease=True)['passed']
    data[2]['wheel_v']=.1
    assert not assess_fault(data,0,2,native_lease=True)['passed']


@pytest.mark.parametrize('failure',['zero','stale','missing_age','negative_age','nan'])
def test_native_expiry_rejects_prior_nonzero_then_zero_or_stale_in_stop_window(failure):
    data=samples()
    for sample in data:sample['safe_v']=.1
    # Earlier live commands are insufficient if ROS zeros or disappears at any
    # physical-stop observation sample. This must not claim a native-lease pass.
    sample=data[2]
    if failure=='zero':sample['safe_v']=0.
    elif failure=='stale':sample['safe_age_s']=.5
    elif failure=='missing_age':sample.pop('safe_age_s')
    elif failure=='negative_age':sample['safe_age_s']=-.1
    else:sample['safe_v']=math.nan
    result=assess_fault(data,0,2,native_lease=True)
    assert result['physical_stop']
    assert not result['commands_remained_live']
    assert not result['passed']


def test_native_expiry_rejects_review_counterexample_early_nonzero_then_watchdog_zero():
    data=samples()
    for sample in data:sample['safe_v']=0.
    data.insert(0,dict(data[0],wall=.6,safe_v=.1,safe_age_s=.05))
    assert not assess_fault(data,0,2,native_lease=True)['passed']


def test_action_success_at_wrong_truth_endpoint_cannot_pass_fixed_alignment():
    alignment=fixed_alignment((0,0,0),(-3,1,.2));goal=(1.,0.,.1)
    truth={'xyz':[-3+math.cos(.2),1+math.sin(.2),0.],
           'quaternion':[0,0,math.sin(.3/2),math.cos(.3/2)]}
    assert route_result(4,goal,alignment,truth,True)['passed']
    truth['xyz'][0]+=.6
    assert not route_result(4,goal,alignment,truth,True)['passed']
    truth['xyz'][0]-=.6
    assert not route_result(4,goal,alignment,truth,False)['passed']
    assert not route_result(5,goal,alignment,truth,True)['passed']
