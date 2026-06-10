#!/usr/bin/env python3
"""
ROS 2 Humble Python boilerplate node.

Default behavior:
- Subscribes to a PoseArray topic.
- Republishes the latest PoseArray to another topic.
- Publishes a simple String heartbeat.
- Logs received messages.

Environment variables:
- NODE_NAME: ROS node name. Default: ros_python_boilerplate
- SUB_TOPIC: input PoseArray topic. Default: /cam/pos
- PUB_TOPIC: output PoseArray topic. Default: /robots/pos
- HEARTBEAT_TOPIC: heartbeat String topic. Default: /boilerplate/heartbeat
- RELIABLE_QOS: true/false. Default: true
- TIMER_PERIOD_SEC: heartbeat period. Default: 1.0
"""

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


def env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "y", "on")


def make_qos() -> QoSProfile:
    reliable = env_bool("RELIABLE_QOS", True)
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


class RosBoilerplateNode(Node):
    def __init__(self) -> None:
        node_name = os.getenv("NODE_NAME", "ros_python_boilerplate")
        super().__init__(node_name)

        self.sub_topic = os.getenv("SUB_TOPIC", "/cam/pos")
        self.pub_topic = os.getenv("PUB_TOPIC", "/robots/pos")
        self.heartbeat_topic = os.getenv("HEARTBEAT_TOPIC", "/boilerplate/heartbeat")
        self.timer_period_sec = float(os.getenv("TIMER_PERIOD_SEC", "1.0"))

        qos = make_qos()

        self.pose_pub = self.create_publisher(PoseArray, self.pub_topic, qos)
        self.heartbeat_pub = self.create_publisher(String, self.heartbeat_topic, qos)
        self.pose_sub = self.create_subscription(PoseArray, self.sub_topic, self.pose_callback, qos)

        self.received_count = 0
        self.latest_pose_array: Optional[PoseArray] = None
        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)

        self.get_logger().info(f"Node started: {node_name}")
        self.get_logger().info(f"Subscribing: {self.sub_topic} [geometry_msgs/msg/PoseArray]")
        self.get_logger().info(f"Publishing:  {self.pub_topic} [geometry_msgs/msg/PoseArray]")
        self.get_logger().info(f"Heartbeat:   {self.heartbeat_topic} [std_msgs/msg/String]")

    def pose_callback(self, msg: PoseArray) -> None:
        self.received_count += 1
        self.latest_pose_array = msg
        self.pose_pub.publish(msg)
        pose_count = len(msg.poses)
        frame_id = msg.header.frame_id
        self.get_logger().info(
            f"Received PoseArray #{self.received_count}: poses={pose_count}, frame_id='{frame_id}'. "
            f"Republished to {self.pub_topic}"
        )

    def timer_callback(self) -> None:
        msg = String()
        msg.data = (
            f"alive received_count={self.received_count} "
            f"sub_topic={self.sub_topic} pub_topic={self.pub_topic}"
        )
        self.heartbeat_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RosBoilerplateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
