from setuptools import setup

package_name = 'wheeltec_vision_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/vision_mapping.launch.py',
            'launch/vision_navigation.launch.py',
            'launch/baseline_record.launch.py',
            'launch/capture_runtime.launch.py',
            'launch/vins_camera.launch.py',
            'launch/vins_record.launch.py',
            'launch/vins_replay.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/baseline_qos_overrides.yaml',
            'config/ekf_visual.yaml',
            'config/realsense_d435c.yaml',
            'config/rtabmap_visual.yaml',
            'config/vision_runtime.rviz',
            'config/vins_qos.yaml',
            'config/runtime.yaml',
            'config/visual_nav_baseline.yaml',
        ]),
        ('share/' + package_name + '/behavior_trees', [
            'behavior_trees/navigate_w_replanning_ackermann.xml',
            'behavior_trees/navigate_w_replanning_ackermann_forward.xml',
            'behavior_trees/navigate_through_poses_ackermann.xml',
            'behavior_trees/navigate_through_poses_ackermann_forward.xml',
        ]),
        ('share/' + package_name, ['README.md', 'BASELINE.md', 'VINS.md', 'ACCEPTANCE.md', 'IMPLEMENTATION.md']),
        ('share/' + package_name + '/tools', [
            'tools/build_vins_overlay.sh', 'tools/rtabmap_vins_visual_health.patch',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Isolated RealSense RGB-D visual mapping and Nav2 navigation for WHEELTEC.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hold_odom_tf = wheeltec_vision_nav.hold_odom_tf:main',
            'wheel_odom_tf = wheeltec_vision_nav.wheel_odom_tf:main',
            'check_baseline = wheeltec_vision_nav.check_baseline:main',
            'emitter_pair = wheeltec_vision_nav.emitter_pair:main',
            'vo_watchdog = wheeltec_vision_nav.vo_watchdog:main',
            'depth_validity_gate = wheeltec_vision_nav.depth_validity_gate:main',
            'capture_runtime = wheeltec_vision_nav.capture_runtime:main',
            'inspect_map_db = wheeltec_vision_nav.inspect_map_db:main',
            'ekf_inputs = wheeltec_vision_nav.ekf_inputs:main',
            'vins_stereo = wheeltec_vision_nav.vins_stereo:main',
            'vins_audit = wheeltec_vision_nav.vins_audit:main',
            'vins_make_config = wheeltec_vision_nav.vins_config:main',
            'vins_monitor = wheeltec_vision_nav.vins_monitor:main',
            'vins_metrics = wheeltec_vision_nav.vins_metrics:main',
            'wait_ready = wheeltec_vision_nav.wait_ready:main',
            'runtime_monitor = wheeltec_vision_nav.runtime_monitor:main',
            'runtime_report = wheeltec_vision_nav.runtime_report:main',
            'acceptance_suite = wheeltec_vision_nav.acceptance:main',
        ],
    },
)
