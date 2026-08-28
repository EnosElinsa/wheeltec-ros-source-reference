from setuptools import setup

package_name = 'largemodel'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name, f'{package_name}.behaviors'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        ('share/' + package_name + '/launch',
            ['launch/robot_bridge.launch.py']),
        ('share/' + package_name + '/config',
            ['config/map_mapping.yaml', 'config/param.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='小薇机器人单步动作控制',
    license='MIT',
    entry_points={
        'console_scripts': [
            'action_service = largemodel.action_service:main',
            'action_bridge = largemodel.action_bridge:main',
        ],
    },
)