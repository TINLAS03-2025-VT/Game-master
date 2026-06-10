#!/usr/bin/env python3
"""Small publisher-only self-test for checking ROS networking."""

import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class SelfTestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("ros_boilerplate_self_test")
        self.topic = os.getenv("TEST_TOPIC", "/cam/pos")
        self.count = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(PoseArray, self.topic, qos)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"Publishing test PoseArray to {self.topic}")

    def timer_callback(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        pose = Pose()
        pose.position.x = float(self.count)
        pose.position.y = 0.0
        pose.position.z = 0.0
        yaw = 0.1 * self.count
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        msg.poses.append(pose)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published test PoseArray #{self.count} to {self.topic}")
        self.count += 1


def main() -> None:
    rclpy.init()
    node = SelfTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
