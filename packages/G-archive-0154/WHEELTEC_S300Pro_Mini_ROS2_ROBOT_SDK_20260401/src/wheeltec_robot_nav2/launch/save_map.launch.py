import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    # 获取 wheeltec_nav2 包的路径
    pkg_share = get_package_share_directory('wheeltec_nav2')
    
    # 获取工作空间根目录 (从 install/pkg/share 往上三级到工作空间根目录)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    
    # 打印路径以便调试
    print(f"Package share: {pkg_share}")
    print(f"Workspace root: {workspace_root}")
    
    # 构建地图保存路径
    install_map_path = os.path.join(pkg_share, 'map', 'WHEELTEC')
    src_map_path = os.path.join(workspace_root, 'src', 'wheeltec_robot_nav2', 'map', 'WHEELTEC')
    
    # 打印最终路径
    print(f"Install map path: {install_map_path}")
    print(f"Source map path: {src_map_path}")
    
    # 确保目录存在
    os.makedirs(os.path.dirname(install_map_path), exist_ok=True)
    os.makedirs(os.path.dirname(src_map_path), exist_ok=True)
    
    # 创建节点
    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', install_map_path],
        parameters=[{'save_map_timeout': 20000.0},
                   {'free_thresh_default': 0.196}]
    )
    
    map_backup = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_backup',
        output='screen',
        arguments=['-f', src_map_path],
        parameters=[{'save_map_timeout': 20000.0},
                   {'free_thresh_default': 0.196}]
    )
    
    return LaunchDescription([map_saver, map_backup])