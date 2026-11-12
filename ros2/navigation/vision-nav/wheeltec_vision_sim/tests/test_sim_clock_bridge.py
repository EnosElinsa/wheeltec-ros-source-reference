"""Inspect pure launch contracts without substituting fake ROS runtime modules."""
import ast
from pathlib import Path

def test_only_sampled_clock_is_bridged_to_ros_clock():
    launch=Path(__file__).resolve().parents[1]/'wheeltec_vision_sim/launching.py'
    tree=ast.parse(launch.read_text())
    functions=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ('bridge_specs','bridge_remappings')]
    scope={};exec(compile(ast.Module(body=functions,type_ignores=[]),str(launch),'exec'),scope)
    clocks=[spec for spec in scope['bridge_specs']() if 'rosgraph_msgs/msg/Clock' in spec]
    assert clocks==['/sim/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    assert ('/sim/clock','/clock') in scope['bridge_remappings']()
    assert not any(spec.startswith('/clock@') for spec in scope['bridge_specs']())


def test_ros_adapter_state_does_not_shadow_humble_readonly_node_collections():
    reserved={'clients','publishers','subscriptions','services','timers','guards','context','executor','default_callback_group'}
    root=Path(__file__).resolve().parents[1]/'wheeltec_vision_sim'
    for path in root.glob('*.py'):
        tree=ast.parse(path.read_text())
        assigned={node.attr for node in ast.walk(tree) if isinstance(node,ast.Attribute)
                  and isinstance(node.ctx,ast.Store) and isinstance(node.value,ast.Name) and node.value.id=='self'}
        assert not assigned&reserved,f'{path.name}: reserved Node state {assigned&reserved}'
