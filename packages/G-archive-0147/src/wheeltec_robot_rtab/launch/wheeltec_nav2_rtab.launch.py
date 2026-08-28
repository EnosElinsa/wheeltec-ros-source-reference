import os, yaml, tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)
def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    my_nav_dir = get_package_share_directory('wheeltec_nav2')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(my_nav_dir, 'map', 'WHEELTEC.yaml'))
        
    rtabmap_nav_dir = get_package_share_directory('wheeltec_robot_rtab')
  
    wheeltec_bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_nav2_dir =  get_package_share_directory('wheeltec_nav2')

    cfg_params = load_yaml(os.path.join(wheeltec_bringup_dir,'config','wheeltec_param.yaml'))
    car_mode = cfg_params['car_mode']
    print(f"car_mode:{car_mode}")
    
    nav2_param_path = os.path.join(wheeltec_nav2_dir,'param','wheeltec_params',f'param_{car_mode}.yaml')
    nav_params = load_yaml(nav2_param_path)
    
    for k in ['amcl', 'amcl_map_client', 'amcl_rclcpp_node', 'map_server', 'map_saver']:
        nav_params.pop(k, None)

    filtered_param_path = os.path.join(rtabmap_nav_dir, 'params', 'rtab_nav2param_filtered.yaml')
    with open(filtered_param_path, 'w') as f:
        yaml.dump(nav_params, f, allow_unicode=True)

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch', 'wheeltec_camera.launch.py')),
    )
    wheeltec_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rtabmap_nav_dir, 'launch', 'rtabmap_localization.launch.py')),
    )    

    time_dec = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true')
    map_dec = DeclareLaunchArgument('map', default_value=map_yaml_path,
                              description='Full path to map file to load')
   
    nav_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={      
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': filtered_param_path}.items(),
        ) 
    waypoint_cycle = Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        )
    # Create the launch description and populate
    ld = LaunchDescription()

    #wheeltec sensors
    ld.add_action(wheeltec_robot)
    ld.add_action(wheeltec_lidar)
    ld.add_action(wheeltec_camera)
    #wheeltec_localization
    #ld.add_action(wheeltec_localization)
    # Declare the launch options
    ld.add_action(time_dec)
    ld.add_action(map_dec)
    ld.add_action(nav_bringup)
    ld.add_action(waypoint_cycle)

    return ld
