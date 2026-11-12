"""Simulation ownership: shared database locks, transport-partition world locks."""
import os

from wheeltec_vision_nav.startup import ResourceLeases, database_resource


def acquire_simulation_leases(database):
    # Runtime containers set a bind-mounted directory so all launchers lock the
    # same persistent inodes. Never unlink lock files when releasing ownership.
    root = os.environ.get('WHEELTEC_SIM_LEASE_ROOT') or None
    partition = os.environ.get('IGN_PARTITION', '')
    return ResourceLeases(root).acquire([
        database_resource(str(database)), f'gazebo:{partition}:wheeltec_room'])
