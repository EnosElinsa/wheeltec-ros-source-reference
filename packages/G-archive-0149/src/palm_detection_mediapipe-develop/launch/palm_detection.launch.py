# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    image_width_launch_arg = DeclareLaunchArgument(
        "palm_det_width", default_value=TextSubstitution(text="640")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "palm_det_height", default_value=TextSubstitution(text="480")
    )
    score_type_launch_arg = DeclareLaunchArgument(
        "palm_min_score", default_value=TextSubstitution(text="0.6")
    )
    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)

    camera_node = None
    camera_type_mipi = None
    camera_device_arg = None

    if camera_type == "fb":
        print("using feedback")
        # local image publish
        fb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_image_publisher'),
                    'launch/hobot_image_publisher.launch.py')),
            launch_arguments={
                'publish_message_topic_name': '/image',
                'publish_is_shared_mem': 'False',
                'publish_is_compressed_img_pub': 'True',
                'publish_output_image_w': LaunchConfiguration('palm_det_width'),
                'publish_output_image_h': LaunchConfiguration('palm_det_height')
            }.items()
        )
        camera_node = fb_node
        camera_type_mipi = False
    elif camera_type == "usb":
        print("using usb camera")
        # using usb cam publish image
        usb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_usb_cam'),
                    'launch/hobot_usb_cam.launch.py')),
            launch_arguments={
                'usb_image_width': LaunchConfiguration('palm_det_width'),
                'usb_image_height': LaunchConfiguration('palm_det_height')
            }.items()
        )

        camera_node = usb_node
        camera_type_mipi = False
    else:
        print("using mipi cam")
        # using mipi cam publish image
        mipi_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='F37',
            description='mipi camera device')

        mipi_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mipi_cam'),
                    'launch/mipi_cam.launch.py')),
            launch_arguments={
                'mipi_image_width': LaunchConfiguration('palm_det_width'),
                'mipi_image_height': LaunchConfiguration('palm_det_height'),
                'mipi_io_method': 'shared_mem',
                'mipi_frame_ts_type': 'realtime',
                'mipi_video_device': LaunchConfiguration('device')
            }.items()
        )

        camera_node = mipi_node
        camera_type_mipi = True
        camera_device_arg = mipi_cam_device_arg

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image'
        }.items()
    )

    # jpeg->nv12
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    # web
    web_smart_topic_arg = DeclareLaunchArgument(
        'smart_topic',
        default_value='/hobot_palm_detection',
        description='websocket smart topic')
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_smart_topic': LaunchConfiguration('smart_topic')
        }.items()
    )

    # palm detection
    palm_det_pub_topic_arg = DeclareLaunchArgument(
        'palm_det_pub_topic',
        default_value='/hobot_palm_detection',
        description='palm detction ai message publish topic')
    mono2d_palm_det_node = Node(
        package='palm_detection_mediapipe',
        executable='palm_detection_mediapipe',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": LaunchConfiguration('palm_det_pub_topic')},
            {"min_score": LaunchConfiguration('palm_min_score')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    shared_mem_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hobot_shm'),
                        'launch/hobot_shm.launch.py'))
            )

    if camera_type_mipi:
        return LaunchDescription([
            image_width_launch_arg,
            image_height_launch_arg,
            score_type_launch_arg,
            camera_device_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # image publish
            camera_node,
            # image codec
            jpeg_codec_node,
            # palm detection
            palm_det_pub_topic_arg,
            mono2d_palm_det_node,
            # web display
            web_smart_topic_arg,
            web_node
        ])
    else:
        return LaunchDescription([
            image_width_launch_arg,
            image_height_launch_arg,
            score_type_launch_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # image publish
            camera_node,
            # image codec
            nv12_codec_node,
            # palm detection
            palm_det_pub_topic_arg,
            mono2d_palm_det_node,
            # web display
            web_smart_topic_arg,
            web_node
        ])
