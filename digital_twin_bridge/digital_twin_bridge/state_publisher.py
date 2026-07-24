#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dt_state_publisher — ammr -> IsaacSim 상태 송신 노드.

디지털 트윈(IsaacSim)에서 실물 로봇의 자세와 속도를 시각화/동기화하기 위해,
map 프레임 기준 localized pose 와 속도를 하나의 깔끔한 토픽으로 내보낸다.

- 위치/방향(x, y, z, orientation):
    1순위) TF (map -> base_footprint) 30Hz 조회. 주행 중 amcl 이 map->odom 을,
           오도메트리가 odom->base_footprint 를 갱신하므로 가장 연속적/정확하다.
    2순위) TF 에 map 프레임이 아직 없을 때(정지 등)는 최신 /amcl_pose (map 프레임)로 대체.
           -> 항상 map 프레임 pose 를 내보내도록 보장한다.
- 선속도/각속도: /odom (nav_msgs/Odometry) 의 twist 를 그대로 사용.

발행:
  /ammr/state  (nav_msgs/Odometry)      : pose(map) + twist(base) 통합 — IsaacSim 권장 구독 대상
  /ammr/pose   (geometry_msgs/PoseStamped)  : 편의용 map 프레임 자세
  /ammr/twist  (geometry_msgs/TwistStamped) : 편의용 속도

기존 자율주행 노드에는 전혀 개입하지 않는다(구독만 함).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped

import tf2_ros
from tf2_ros import TransformException


class StatePublisher(Node):
    def __init__(self):
        super().__init__('dt_state_publisher')

        # ---- 파라미터 ----
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('publish_rate_hz', 30.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        amcl_topic = self.get_parameter('amcl_pose_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 최신 상태 보관 ----
        self._last_twist = None       # /odom twist
        self._last_amcl_pose = None   # /amcl_pose 의 map 프레임 pose (fallback)

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, odom_topic, self._odom_cb, odom_qos)
        # amcl_pose 는 RELIABLE + TRANSIENT_LOCAL(latched) 로 발행 -> QoS 를 맞춰야
        # 마지막 latched 값을 수신할 수 있다(정지 상태에서도 pose 확보).
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_topic, self._amcl_cb, amcl_qos)

        # ---- 발행자 ----
        self.state_pub = self.create_publisher(Odometry, '/ammr/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ammr/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/ammr/twist', 10)

        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        self._warned = False
        self.get_logger().info(
            f'dt_state_publisher 시작: TF {self.map_frame}->{self.base_frame}, '
            f'twist<-{odom_topic}, {rate:.0f}Hz 로 /ammr/state 발행'
        )

    def _odom_cb(self, msg: Odometry):
        self._last_twist = msg.twist.twist

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._last_amcl_pose = msg.pose.pose

    def _get_map_pose(self):
        """map 프레임 pose 를 (position, orientation) 로 반환. 없으면 None."""
        # 1순위: TF map->base_footprint (주행 중 연속적)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return tf.transform.translation, tf.transform.rotation
        except TransformException:
            pass
        # 2순위: 최신 /amcl_pose (정지 등 map->odom TF 가 아직 없을 때)
        if self._last_amcl_pose is not None:
            return self._last_amcl_pose.position, self._last_amcl_pose.orientation
        return None

    def _on_timer(self):
        pose = self._get_map_pose()
        if pose is None:
            if not self._warned:
                self.get_logger().warn(
                    f'{self.map_frame} 프레임 pose 없음(TF/amcl_pose 대기중). '
                    '로봇 위치추정이 시작되면 자동으로 발행됩니다.')
                self._warned = True
            return
        if self._warned:
            self.get_logger().info(f'{self.map_frame} 프레임 pose 확보 - /ammr/state 발행 시작')
            self._warned = False

        t, q = pose
        now = self.get_clock().now().to_msg()

        # --- /ammr/state (Odometry) ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = t.x
        odom.pose.pose.position.y = t.y
        odom.pose.pose.position.z = t.z
        odom.pose.pose.orientation = q
        if self._last_twist is not None:
            odom.twist.twist = self._last_twist
        self.state_pub.publish(odom)

        # --- /ammr/pose ---
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        self.pose_pub.publish(ps)

        # --- /ammr/twist ---
        if self._last_twist is not None:
            ts = TwistStamped()
            ts.header.stamp = now
            ts.header.frame_id = self.base_frame
            ts.twist = self._last_twist
            self.twist_pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
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
