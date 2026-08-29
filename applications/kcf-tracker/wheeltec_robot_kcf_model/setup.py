from setuptools import setup

package_name = 'wheeltec_robot_kcf_model'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/kcf_tracker.launch.py']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name, ['wheeltec_robot_kcf_model/yolo_detector.py']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='powrbv@gmail.com',
    description='Python KCF tracker — outputs target XY coordinates for following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kcf_tracker = wheeltec_robot_kcf_model.kcf_tracker_node:main',
        ],
    },
)
