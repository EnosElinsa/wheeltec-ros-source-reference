import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions


def generate_launch_description():
    """Generate a launch description for a single serial driver."""
    config_file = os.path.join(get_package_share_directory("common_wheeltec_sensor_launch"), "params", "nmea_serial_driver.yaml")
    driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        remappings=[('/gps/fix','nav_sat_fix'),
        ('/gps/vel','fix_velocity')],
        parameters=[config_file])

    base_to_gnss = actions.Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_gnss',
        arguments=['0', '0', '0','0', '0','0','base_link','navsat_link'],
        )

    return LaunchDescription([driver_node,base_to_gnss])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)
