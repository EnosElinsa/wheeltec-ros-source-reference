from types import SimpleNamespace as Item
from wheeltec_vision_sim.contacts import collision_count,is_floor
from wheeltec_vision_sim.model import world_sdf
from wheeltec_vision_sim.mission_logic import Evidence,CONTACT_SOURCES
import xml.etree.ElementTree as ET

def contact(a,b):return Item(collision1=Item(name=a),collision2=Item(name=b))

def test_only_actual_generated_floor_support_is_exempt():
    wheel='wheeltec::rear_left_wheel::rear_left_collision'
    assert collision_count([contact(wheel,'floor::link::floor_collision')])==0
    assert collision_count([contact('wheeltec_room::floor::link::floor_collision',wheel)])==0
    assert collision_count([contact(wheel,'floor_obstacle::link::box')])==1
    assert collision_count([contact(wheel,'floor::other_link::floor_collision')])==1
    assert collision_count([contact(wheel,'')])==1
    assert not is_floor('floor_collision')

def test_empty_physics_heartbeat_counts_as_fresh_only_for_observed_sources():
    evidence=Evidence()
    for name in CONTACT_SOURCES:
        evidence.note(name,1_000_000_000,10.,collision_count([]))
    assert all(evidence.snapshot(1_010_000_000,10.1)['evidence_fresh'][n] for n in CONTACT_SOURCES)
    assert evidence.collision_events==0
    assert not evidence.fresh('gt',1_010_000_000,10.1)
    assert not any(evidence.snapshot(2_000_000_000,11.)['evidence_fresh'][n] for n in CONTACT_SOURCES)

def test_contact_capture_collisions_exist_and_stock_event_only_publisher_absent():
    root=ET.fromstring(world_sdf());world=root.find('world');model=world.find("model[@name='wheeltec']")
    assert not any(p.attrib['filename']=='ignition-gazebo-contact-system' for p in world.findall('plugin'))
    for link,collision in [('base_footprint','chassis_collision'),('rear_left_wheel','rear_left_collision'),
          ('rear_right_wheel','rear_right_collision'),('front_left_wheel','front_left_collision'),('front_right_wheel','front_right_collision')]:
        assert model.find(f"link[@name='{link}']/collision[@name='{collision}']") is not None
