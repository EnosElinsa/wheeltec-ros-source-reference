from glob import glob
from setuptools import setup

package_name = 'wheeltec_vision_benchmark'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml', 'README.md']),
                ('share/' + package_name + '/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools', 'numpy', 'PyYAML', 'matplotlib'], zip_safe=True,
    maintainer='Wheeltec maintainers', maintainer_email='maintainers@example.com',
    description='Isolated EuRoC stereo VO/VIO benchmark with independent SE(3) scoring',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'validate_euroc = wheeltec_vision_benchmark.validate:main',
        'euroc_player = wheeltec_vision_benchmark.player:main',
        'trajectory_evaluate = wheeltec_vision_benchmark.trajectory:main']},
)
