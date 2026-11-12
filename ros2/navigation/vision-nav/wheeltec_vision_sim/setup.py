from setuptools import setup
from glob import glob
package_name = 'wheeltec_vision_sim'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[
 ('share/ament_index/resource_index/packages',['resource/'+package_name]),
 ('share/'+package_name,['package.xml','README.md']),
 ('share/'+package_name+'/launch',glob('launch/*.launch.py')),
 ('share/'+package_name+'/worlds',glob('worlds/*')),
 ('share/'+package_name+'/models',glob('models/*')),
 ('share/'+package_name+'/config',glob('config/*')),
], entry_points={'console_scripts':['wheel_odom=wheeltec_vision_sim.wheel_odom:main','sim_inputs=wheeltec_vision_sim.inputs:main','odom_adapter=wheeltec_vision_sim.odom_adapter:main','mission_runner=wheeltec_vision_sim.mission_runner:main']})
