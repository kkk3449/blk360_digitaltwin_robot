#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dt_goal_relay — IsaacSim -> ammr 목표위치 수신 노드.

IsaacSim(디지털 트윈)에서 지정한 목표 위치를 실물 로봇 자율주행에 주입한다.
외부(IsaacSim)용 토픽 /ammr/goal_pose 를 구독하여, ammr 내부 자율주행이
이미 사용 중인 네이티브 토픽 /goal_pose 로 그대로 전달한다.

이렇게 한 단계 릴레이를 두는 이유:
  1) 외부 인터페이스(/ammr/goal_pose)와 내부 토픽(/goal_pose)을 분리 → 안전.
  2) frame_id 보정, 로깅, 향후 검증/필터를 넣을 지점 확보.

기존 자율주행 로직(amr_interface 의 goal_pose 콜백 -> NavigateToPose)은 그대로 사용.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalRelay(Node):
    def __init__(self):
        super().__init__('dt_goal_relay')

        self.declare_parameter('in_topic', '/ammr/goal_pose')
        self.declare_parameter('out_topic', '/goal_pose')
        self.declare_parameter('map_frame', 'map')

        in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.map_frame = self.get_parameter('map_frame').value

        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.create_subscription(PoseStamped, in_topic, self._cb, 10)

        self.get_logger().info(
            f'dt_goal_relay 시작: {in_topic}  ->  {self.out_topic} (frame={self.map_frame})')

    def _cb(self, msg: PoseStamped):
        out = PoseStamped()
        out.pose = msg.pose
        # frame_id 가 비어오면 map 으로 보정(자율주행은 map 프레임 기준).
        out.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(out)
        p = out.pose.position
        self.get_logger().info(
            f'목표 수신 -> 전달: x={p.x:.3f}, y={p.y:.3f}, frame={out.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
