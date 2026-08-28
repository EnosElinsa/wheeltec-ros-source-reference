import os
from setuptools import find_packages, setup

package_name = 'wheeltec_jetracer'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/wheeltec_jetracer.launch.py']))
data_files.append((os.path.join('share', package_name, 'param'), ['param/v550_akm.yaml']))
data_files.append((os.path.join('share', package_name, 'param'), ['param/v550_mec.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools','launch'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_following = wheeltec_jetracer.road_following:main',
            'laser_detect = wheeltec_jetracer.laser_detect:main',
            'utils = wheeltec_jetracer.utils:main'
        ],
    },
)
