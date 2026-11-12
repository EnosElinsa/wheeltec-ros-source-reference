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
