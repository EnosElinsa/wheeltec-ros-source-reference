"""Nav2 action discovery is insufficient until managed nodes are active."""
from types import SimpleNamespace as NS

import pytest

from wheeltec_vision_sim.mission_runner import NAVIGATION_NODES, wait_navigation_active


class Clock:
    now = 0.
    def __call__(self): return self.now
    def spin(self, duration): self.now += duration


class Client:
    def __init__(self, clock, active_at=0., service=True, reply=True):
        self.clock, self.active_at, self.service, self.reply = clock, active_at, service, reply
        self.calls = 0
    def service_is_ready(self): return self.service
    def call_async(self, request):
        self.calls += 1
        state = 3 if self.clock() >= self.active_at else 2
        return NS(done=lambda: self.reply, result=lambda: NS(current_state=NS(id=state)))


def test_waits_for_every_managed_node_active_after_discovery():
    clock = Clock()
    clients = {name: Client(clock) for name in NAVIGATION_NODES}
    clients['bt_navigator'].active_at = .8
    wait_navigation_active(clients, object, clock.spin, timeout=2., clock=clock)
    assert clock() >= .8
    assert clients['bt_navigator'].calls > 1
    assert all(client.calls > 1 for client in clients.values())


@pytest.mark.parametrize('failure', ['inactive', 'missing_service', 'no_reply'])
def test_inactive_or_unresponsive_node_blocks_before_first_goal(failure):
    clock = Clock()
    clients = {name: Client(clock) for name in NAVIGATION_NODES}
    clients['bt_navigator'] = Client(clock, active_at=10. if failure=='inactive' else 0.,
        service=failure!='missing_service', reply=failure!='no_reply')
    with pytest.raises(RuntimeError, match='bt_navigator='):
        wait_navigation_active(clients, object, clock.spin, timeout=.8, clock=clock)
    assert clock() < .9


def test_cached_active_response_expires_while_other_nodes_activate():
    clock = Clock()
    clients = {name: Client(clock) for name in NAVIGATION_NODES}
    clients['bt_navigator'].active_at = 1.
    def spin(duration):
        clock.spin(duration)
        if clock() > .1:
            clients['controller_server'].service = False
    with pytest.raises(RuntimeError, match='all Nav2 lifecycle nodes ACTIVE'):
        wait_navigation_active(clients, object, spin, timeout=1.5, clock=clock)


def test_late_active_reply_cannot_be_treated_as_a_current_snapshot():
    clock = Clock()
    class DelayedClient(Client):
        def call_async(self, request):
            self.calls += 1
            started = self.clock()
            return NS(done=lambda: self.clock()-started > .6,
                      result=lambda: NS(current_state=NS(id=3)))
    clients = {name: DelayedClient(clock) for name in NAVIGATION_NODES}
    with pytest.raises(RuntimeError, match='all Nav2 lifecycle nodes ACTIVE'):
        wait_navigation_active(clients, object, clock.spin, timeout=1.5, clock=clock)


@pytest.mark.parametrize('failure', ['never_available', 'motion', 'stale'])
def test_exact_initial_alignment_wait_stays_bounded_and_rejects_motion_or_stale_pose(failure):
    from wheeltec_vision_sim.mission_runner import wait_exact_transform
    clock=Clock();requested=[]
    class Missing(Exception): pass
    def lookup(stamp):
        requested.append(stamp)
        raise Missing()
    valid=lambda:clock()<(.02 if failure in ('motion','stale') else 2.)
    with pytest.raises(RuntimeError,match='fresh stationary exact-stamp TF'):
        wait_exact_transform(lookup,123,clock.spin,valid,Missing,clock=clock)
    assert set(requested)=={123} and clock()<=.31


def test_exact_initial_alignment_wait_retries_one_frozen_acquisition_while_spinning():
    from wheeltec_vision_sim.mission_runner import wait_exact_transform
    clock=Clock();requested=[]
    class Missing(Exception): pass
    def lookup(stamp):
        requested.append(stamp)
        if clock()<.04:raise Missing()
        return 'exact transform'
    result=wait_exact_transform(lookup,123,clock.spin,lambda:True,Missing,clock=clock)
    assert result=='exact transform' and requested==[123]*len(requested) and len(requested)>1


def test_stationary_alignment_requires_every_twist_axis_finite_and_small():
    from wheeltec_vision_sim.mission_runner import stationary_twist
    msg=NS(twist=NS(twist=NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))))
    assert stationary_twist(msg)
    for vector in (msg.twist.twist.linear,msg.twist.twist.angular):
        for axis in ('x','y','z'):
            setattr(vector,axis,.1);assert not stationary_twist(msg);setattr(vector,axis,0.)
            setattr(vector,axis,float('nan'));assert not stationary_twist(msg);setattr(vector,axis,0.)
