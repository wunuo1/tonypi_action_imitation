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

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    usb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_usb_cam'),
                'launch/hobot_usb_cam.launch.py')),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_pixel_format': 'yuyv2rgb',
            'usb_zero_copy': 'False',
            'usb_framerate': '10',
        }.items()
    )

    tonypi_image_correction = Node(
        package='tonypi_image_correction',
        executable='tonypi_image_correction',
        parameters=[
            {"pub_image_topic": 'hbmem_img'},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # nv12->jpeg
    jpeg_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"in_mode": 'shared_mem'},
            {"in_format": "nv12"},
            {"out_mode": 'ros'},
            {"out_format": "jpeg"},
            {"sub_topic": 'hbmem_img'},
            {"dump_output": False},
            {"pub_topic": 'image_jpeg'},
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_smart_topic': '/hobot_hand_gesture_detection'
        }.items()
    )

    mono2d_body_det_node = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": '/hobot_mono2d_body_detection'}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    hand_lmk_pub_topic_arg = DeclareLaunchArgument(
        'hand_lmk_pub_topic',
        default_value='/hobot_hand_lmk_detection',
        description='hand landmark ai message publish topic')

    hand_lmk_det_node = Node(
        package='hand_lmk_detection',
        executable='hand_lmk_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": LaunchConfiguration(
                'hand_lmk_pub_topic')},
            {"ai_msg_sub_topic_name": "/hobot_mono2d_body_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 手势识别算法
    hand_gesture_det_node = Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": "/hobot_hand_gesture_detection"},
            {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    tonypi_action_imitation_node = Node(
        package='tonypi_action_imitation',
        executable='tonypi_action_imitation',
        output='screen',
        parameters=[
            {"pluse": 1500},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )



    return LaunchDescription([
        usb_node,
        # 启动零拷贝环境配置node
        # shared_mem_node,
        tonypi_image_correction,
        # image publish
        jpeg_codec_node,
        mono2d_body_det_node,
        hand_lmk_pub_topic_arg,
        hand_lmk_det_node,
        hand_gesture_det_node,
        tonypi_action_imitation_node,
        # web display
        web_node
    ])
