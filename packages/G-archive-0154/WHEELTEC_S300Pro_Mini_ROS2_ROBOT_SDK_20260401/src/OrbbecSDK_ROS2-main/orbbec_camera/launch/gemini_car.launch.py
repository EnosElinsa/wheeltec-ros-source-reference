from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare arguments
    args = [
	DeclareLaunchArgument('camera_name', default_value='camera'),  # 相机命名空间/前缀；用于组装话题与frame_id
	DeclareLaunchArgument('depth_registration', default_value='true'),  # 是否将深度注册到彩色相机坐标（D2C对齐）
	DeclareLaunchArgument('serial_number', default_value='AY2M543030A'),  # 设备序列号筛选；空=不按SN筛选
	DeclareLaunchArgument('usb_port', default_value=''),  # USB 端口路径筛选（如 '2-1'）；空=不按端口筛选
	DeclareLaunchArgument('device_num', default_value='1'),  # 多设备时的序号选择（从1开始）；无SN/端口时生效
	DeclareLaunchArgument('vendor_id', default_value='0x2bc5'),  # USB Vendor ID（0x2bc5 常见为 Orbbec）
	DeclareLaunchArgument('product_id', default_value=''),  # USB Product ID；空=接受该厂商所有产品
	DeclareLaunchArgument('enable_point_cloud', default_value='true'),  # 发布点云（由深度/内参生成）
	DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),  # 点云着色（使用彩色图给点云上色）
	DeclareLaunchArgument('cloud_frame_id', default_value=''),  # 点云frame_id；空=使用默认相机链接frame
	DeclareLaunchArgument('point_cloud_qos', default_value='default'),  # 点云话题QoS（如 'default'/'sensor_data'）
	DeclareLaunchArgument('connection_delay', default_value='100'),  # 启动后延时连接设备（毫秒）
	DeclareLaunchArgument('color_width', default_value='640'),  # 彩色分辨率宽
	DeclareLaunchArgument('color_height', default_value='480'),  # 彩色分辨率高
	DeclareLaunchArgument('color_fps', default_value='30'),  # 彩色帧率
	DeclareLaunchArgument('color_format', default_value='MJPG'),  # 彩色流格式（如 'MJPG'/'YUYV'）
	DeclareLaunchArgument('enable_color', default_value='true'),  # 启用彩色流
	DeclareLaunchArgument('flip_color', default_value='false'),  # 彩色图左右翻转
	DeclareLaunchArgument('color_qos', default_value='default'),  # 彩色图像话题QoS
	DeclareLaunchArgument('color_camera_info_qos', default_value='default'),  # 彩色CameraInfo话题QoS
	DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),  # 彩色自动曝光
	DeclareLaunchArgument('color_exposure', default_value='-1'),  # 彩色手动曝光；-1=跟随自动
	DeclareLaunchArgument('color_gain', default_value='-1'),  # 彩色手动增益；-1=跟随自动
	DeclareLaunchArgument('enable_color_auto_white_balance', default_value='true'),  # 彩色自动白平衡
	DeclareLaunchArgument('color_white_balance', default_value='-1'),  # 彩色手动白平衡（开尔文）；-1=自动
	DeclareLaunchArgument('depth_width', default_value='640'),  # 深度分辨率宽
	DeclareLaunchArgument('depth_height', default_value='400'),  # 深度分辨率高
	DeclareLaunchArgument('depth_fps', default_value='30'),  # 深度帧率
	DeclareLaunchArgument('depth_format', default_value='Y11'),  # 深度流格式（设备定义，常见为Y11/Y16）
	DeclareLaunchArgument('enable_depth', default_value='true'),  # 启用深度流
	DeclareLaunchArgument('flip_depth', default_value='false'),  # 深度图左右翻转
	DeclareLaunchArgument('depth_qos', default_value='default'),  # 深度图像话题QoS
	DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),  # 深度CameraInfo话题QoS
	DeclareLaunchArgument('ir_width', default_value='640'),  # 红外分辨率宽
	DeclareLaunchArgument('ir_height', default_value='400'),  # 红外分辨率高
	DeclareLaunchArgument('ir_fps', default_value='30'),  # 红外帧率
	DeclareLaunchArgument('ir_format', default_value='Y10'),  # 红外流格式（如 Y10/Y16）
	DeclareLaunchArgument('enable_ir', default_value='true'),  # 启用红外流
	DeclareLaunchArgument('flip_ir', default_value='false'),  # 红外图左右翻转
	DeclareLaunchArgument('ir_qos', default_value='default'),  # 红外图像话题QoS
	DeclareLaunchArgument('ir_camera_info_qos', default_value='default'),  # 红外CameraInfo话题QoS
	DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),  # 红外自动曝光
	DeclareLaunchArgument('ir_exposure', default_value='-1'),  # 红外手动曝光；-1=跟随自动
	DeclareLaunchArgument('ir_gain', default_value='-1'),  # 红外手动增益；-1=跟随自动
	DeclareLaunchArgument('publish_tf', default_value='true'),  # 发布相机坐标系TF（静态/动态）
	DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),  # TF发布频率Hz；0.0=仅一次或按设备策略
	DeclareLaunchArgument('ir_info_url', default_value=''),  # IR相机标定YAML URL（camera_info_manager）
	DeclareLaunchArgument('color_info_url', default_value=''),  # 彩色相机标定YAML URL
	DeclareLaunchArgument('log_level', default_value='none'),  # 日志级别（none/info/warn/debug等）
	DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),  # 发布传感器间外参（如 depth↔color）
	DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),  # 启用D2C预览/可视化（驱动实现相关）
	DeclareLaunchArgument('enable_ldp', default_value='true'),  # 设备特性：激光/投射相关保护/调光模式（厂商定义）
	DeclareLaunchArgument('enable_soft_filter', default_value='true'),  # 启用深度软件滤波（去噪/去散斑）
	DeclareLaunchArgument('soft_filter_max_diff', default_value='-1'),  # 滤波阈值：相邻深度最大差；-1=默认
	DeclareLaunchArgument('soft_filter_speckle_size', default_value='-1'),  # 最小散斑尺寸；-1=默认
	DeclareLaunchArgument('ordered_pc', default_value='false'),  # 点云是否保持图像网格顺序（有序点云）
	DeclareLaunchArgument('use_hardware_time', default_value='false'),  # 使用设备硬件时间戳作为消息时间
	DeclareLaunchArgument('enable_depth_scale', default_value='true'),  # 应用/发布深度尺度（将原始单位换算为米）
	DeclareLaunchArgument('align_mode', default_value='HW'),  # 对齐模式：'HW'硬件对齐（如 D2C），或 'SW'/'NONE'
	DeclareLaunchArgument('laser_energy_level', default_value='-1'),  # 激光/投射能量等级；-1=自动（范围依设备）
	DeclareLaunchArgument('enable_heartbeat', default_value='false'),  # 启用心跳保活（防止设备休眠/断流）
    ]

    # Node configuration
    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]
    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "foxy":
        return LaunchDescription(
            args
            + [
                Node(
                    package="orbbec_camera",
                    executable="orbbec_camera_node",
                    name="ob_camera_node",
                    namespace=LaunchConfiguration("camera_name"),
                    parameters=parameters,
                    output="screen",
                )
            ]
        )
    # Define the ComposableNode
    else:
        # Define the ComposableNode
        compose_node = ComposableNode(
            package="orbbec_camera",
            plugin="orbbec_camera::OBCameraNodeDriver",
            name=LaunchConfiguration("camera_name"),
            namespace="",
            parameters=parameters,
        )
        # Define the ComposableNodeContainer
        container = ComposableNodeContainer(
            name="camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                compose_node,
            ],
            output="screen",
        )
        # Launch description
        ld = LaunchDescription(
            args
            + [
                GroupAction(
                    [PushRosNamespace(LaunchConfiguration("camera_name")), container]
                )
            ]
        )
        return ld
