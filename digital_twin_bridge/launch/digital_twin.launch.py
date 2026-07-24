#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
digital_twin.launch.py — ammr <-> IsaacSim 디지털 트윈 브릿지 실행.

두 노드를 띄운다:
  - dt_state_publisher : ammr -> IsaacSim (map 프레임 pose + 속도 송신, /ammr/state)
  - dt_goal_relay      : IsaacSim -> ammr (목표위치 수신, /ammr/goal_pose -> /goal_pose)

두 프로세스는 브릿지 전용 CycloneDDS 설정(config/cyclonedds_twin.xml)을 사용해
IsaacSim(192.168.31.135)과 같은 wifi 망(192.168.31.x)에서 통신한다.
설정은 각 노드 프로세스에만 additional_env 로 주입되므로, 이미 실행 중인
기존 자율주행 스택(원래 /root/cyclonedds.xml 사용)에는 전혀 영향이 없다.

인자:
  cyclonedds_uri : 브릿지가 사용할 CycloneDDS xml 경로
                   (기본값=이 패키지의 config/cyclonedds_twin.xml)
                   기존 설정으로 돌리려면 cyclonedds_uri:=/root/cyclonedds.xml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('digital_twin_bridge')
    default_dds = os.path.join(pkg_share, 'config', 'cyclonedds_twin.xml')

    dds_uri = LaunchConfiguration('cyclonedds_uri')
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    rate = LaunchConfiguration('rate')

    args = [
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds,
                              description='브릿지 프로세스가 사용할 CycloneDDS xml 경로'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('rate', default_value='30.0',
                              description='상태 발행 주기(Hz)'),
    ]

    # 브릿지 노드에만 CYCLONEDDS_URI 를 주입(기존 스택 환경은 불변).
    twin_env = {'CYCLONEDDS_URI': ['file://', dds_uri]}

    state_node = Node(
        package='digital_twin_bridge',
        executable='dt_state_publisher',
        name='dt_state_publisher',
        output='screen',
        additional_env=twin_env,
        parameters=[{
            'map_frame': map_frame,
            'base_frame': base_frame,
            'odom_topic': '/odom',
            'amcl_pose_topic': '/amcl_pose',
            'publish_rate_hz': rate,
        }],
    )

    goal_node = Node(
        package='digital_twin_bridge',
        executable='dt_goal_relay',
        name='dt_goal_relay',
        output='screen',
        additional_env=twin_env,
        parameters=[{
            'in_topic': '/ammr/goal_pose',
            'out_topic': '/goal_pose',
            'map_frame': map_frame,
        }],
    )

    return LaunchDescription(args + [state_node, goal_node])
