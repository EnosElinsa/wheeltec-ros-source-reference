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
            'wheeltec_vision_nav/launch_utils.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/baseline_qos_overrides.yaml',
            'config/ekf_visual.yaml',
            'config/realsense_d435c.yaml',
            'config/rtabmap_visual.yaml',
            'config/vision_runtime.rviz',
        ]),
        ('share/' + package_name + '/behavior_trees', [
            'behavior_trees/navigate_w_replanning_ackermann.xml',
        ]),
        ('share/' + package_name, ['README.md', 'BASELINE.md']),
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
            'capture_runtime = wheeltec_vision_nav.capture_runtime:main',
            'inspect_map_db = wheeltec_vision_nav.inspect_map_db:main',
        ],
    },
)
