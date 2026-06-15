#!/usr/bin/env python3
"""
Game-master ROS 2 node boilerplate.

This is NOT a relay.

Default behavior:
- Publishes a counter on /game_master/counter
- Publishes a heartbeat on /game_master/heartbeat
- Subscribes to text commands on /game_master/input
- Subscribes to robot positions on /robots/pos

Environment variables:
- NODE_NAME
- COUNTER_TOPIC
- HEARTBEAT_TOPIC
- INPUT_TOPIC
- ROBOTS_POS_TOPIC
- TIMER_PERIOD_SEC
"""

import os

from typing import Sequence, cast

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String, UInt64
from geometry_msgs.msg import PoseArray, Pose

def make_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


class GameMasterNode(Node):
    def __init__(self) -> None:
        node_name = os.getenv("NODE_NAME", "game_master")

        super().__init__(node_name)

        self.counter_topic = os.getenv("COUNTER_TOPIC", "/game_master/counter")
        self.heartbeat_topic = os.getenv("HEARTBEAT_TOPIC", "/game_master/heartbeat")
        self.input_topic = os.getenv("INPUT_TOPIC", "/game_master/input")
        self.robots_pos_topic = os.getenv("ROBOTS_POS_TOPIC", "/robots/pos")
        self.timer_period_sec = float(os.getenv("TIMER_PERIOD_SEC", "1.0"))

        qos = make_qos()

        self.counter_pub = self.create_publisher(UInt64, self.counter_topic, qos)
        self.heartbeat_pub = self.create_publisher(String, self.heartbeat_topic, qos)

        self.input_sub = self.create_subscription(
            String,
            self.input_topic,
            self.input_callback,
            qos,
        )

        self.robots_pos_sub = self.create_subscription(
            PoseArray,
            self.robots_pos_topic,
            self.robots_pos_callback,
            qos,
        )

        self.counter = 0
        self.input_count = 0
        self.posearray_count = 0

        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)

        self.get_logger().info("Game-master node started")
        self.get_logger().info(f"Publishing counter:   {self.counter_topic} [std_msgs/msg/UInt64]")
        self.get_logger().info(f"Publishing heartbeat: {self.heartbeat_topic} [std_msgs/msg/String]")
        self.get_logger().info(f"Subscribing input:    {self.input_topic} [std_msgs/msg/String]")
        self.get_logger().info(f"Subscribing robots:   {self.robots_pos_topic} [geometry_msgs/msg/PoseArray]")

    def timer_callback(self) -> None:
        counter_msg = UInt64()
        counter_msg.data = self.counter
        self.counter_pub.publish(counter_msg)

        heartbeat_msg = String()
        heartbeat_msg.data = (
            f"alive counter={self.counter} "
            f"input_count={self.input_count} "
            f"posearray_count={self.posearray_count}"
        )
        self.heartbeat_pub.publish(heartbeat_msg)

        self.get_logger().info(heartbeat_msg.data)

        self.counter += 1

    def input_callback(self, msg: String) -> None:
        self.input_count += 1
        self.get_logger().info(
            f"Received input #{self.input_count} on {self.input_topic}: {msg.data}"
        )

    def robots_pos_callback(self, msg: PoseArray) -> None:
        self.posearray_count += 1

        pose_count = len(msg.poses)
        frame_id = msg.header.frame_id

        self.get_logger().info(
            f"Received robots PoseArray #{self.posearray_count} "
            f"on {self.robots_pos_topic}: poses={pose_count}, frame_id='{frame_id}'"
        )

        if pose_count > 0:
            poses = cast(Sequence[Pose], msg.poses)
            first = poses[0]
            self.get_logger().info(
                "First robot pose: "
                f"x={first.position.x:.3f}, "
                f"y={first.position.y:.3f}, "
                f"z={first.position.z:.3f}"
            )


def main() -> None:
    rclpy.init()

    node = GameMasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
