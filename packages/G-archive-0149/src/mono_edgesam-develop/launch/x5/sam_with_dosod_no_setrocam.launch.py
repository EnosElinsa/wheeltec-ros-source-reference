# Copyright (c) 2025，D-Robotics.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    basic_path = os.path.join(
        get_package_prefix('mono_edgesam'),
        'lib/mono_edgesam/config')

    print("mono_edgesam basic_path is ", basic_path)

    # args that can be set from the command line or a default will be used
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "sam_msg_pub_topic_name", default_value=TextSubstitution(text="perception/segmentation/edgesam")
    )
    model_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_model_file_name", default_value=TextSubstitution(text="config/dosod_mlp3x_s_rep-int8.bin")
    )
    vocabulary_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_vocabulary_file_name", default_value=TextSubstitution(text="config/offline_vocabulary.json")
    )
    encoder_model_file_name_launch_arg = DeclareLaunchArgument(
        "sam_encoder_model_file_name", default_value=TextSubstitution(text="edgesam_encoder_512.bin")
    )
    decoder_model_file_name_launch_arg = DeclareLaunchArgument(
        "sam_decoder_model_file_name", default_value=TextSubstitution(text="edgesam_decoder_512.bin")
    )

    # 算法pkg
    sam_node = Node(
        package='mono_edgesam',
        executable='mono_edgesam',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_regular_box": 0},
            {"is_padding_seg": 1},
            {"is_shared_mem_sub": 0},
            {"ros_img_sub_topic_name": "/image_left_raw"},
            {"ai_msg_sub_topic_name": "/hobot_dnn_detection"},
            {"encoder_model_file_name": [basic_path, "/", LaunchConfiguration(
                "sam_encoder_model_file_name")]},
            {"decoder_model_file_name": [basic_path, "/", LaunchConfiguration(
                "sam_decoder_model_file_name")]},
            {"msg_pub_topic_name": LaunchConfiguration(
                "sam_msg_pub_topic_name")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 算法pkg
    dosod_node = Node(
        package='hobot_dosod',
        executable='hobot_dosod',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_shared_mem_sub": 0},
            {"ros_img_sub_topic_name": "/image_left_raw"},
            {"roi": False},
            {"dump_raw_img": 0},
            {"dump_render_img": 0},
            {"ai_msg_pub_topic_name": "/hobot_dnn_detection"},
            {"model_file_name": LaunchConfiguration(
                "dosod_model_file_name")},
            {"vocabulary_file_name": LaunchConfiguration(
                "dosod_vocabulary_file_name")},
            {"trigger_mode": 0},
            {"class_mode": 0},
            {"score_threshold": 0.6}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        encoder_model_file_name_launch_arg,
        decoder_model_file_name_launch_arg,
        msg_pub_topic_name_launch_arg,
        model_file_name_launch_arg,
        vocabulary_file_name_launch_arg,
        # 启动dosod pkg
        dosod_node,
        # 启动sam pkg
        sam_node
    ])