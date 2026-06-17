#!/usr/bin/env python3

import os
from copy import deepcopy
from typing import Sequence, cast

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def make_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def robot_id_from_pose(pose: Pose) -> int:
    return int(round(pose.position.z))


class RobotPositionMergerNode(Node):
    def __init__(self) -> None:
        super().__init__(os.getenv("NODE_NAME", "robot_position_merger"))

        self.unity_pos_topic = os.getenv("UNITY_POS_TOPIC", "/unity/pos")
        self.cam_pos_topic = os.getenv("CAM_POS_TOPIC", "/cam/pos")
        self.robots_pos_topic = os.getenv("ROBOTS_POS_TOPIC", "/robots/pos")
        self.output_frame_id = os.getenv("OUTPUT_FRAME_ID", "map")

        qos = make_qos()

        self.robots_pos_pub = self.create_publisher(
            PoseArray,
            self.robots_pos_topic,
            qos,
        )

        self.unity_pos_sub = self.create_subscription(
            PoseArray,
            self.unity_pos_topic,
            self.unity_pos_callback,
            qos,
        )

        self.cam_pos_sub = self.create_subscription(
            PoseArray,
            self.cam_pos_topic,
            self.cam_pos_callback,
            qos,
        )

        self.unity_poses: list[Pose] = []
        self.cam_poses: list[Pose] = []

        self.have_unity = False
        self.have_cam = False

        self.publish_count = 0
        self.unity_count = 0
        self.cam_count = 0

        self.get_logger().info("Robot position merger started")
        self.get_logger().info(f"Subscribing Unity poses: {self.unity_pos_topic}")
        self.get_logger().info(f"Subscribing cam poses:   {self.cam_pos_topic}")
        self.get_logger().info(f"Publishing merged poses: {self.robots_pos_topic}")
        self.get_logger().info("Merge order: unity first, then cam")
        self.get_logger().info("No age timeout. Last known poses are reused until replaced.")

        if self.robots_pos_topic in [self.unity_pos_topic, self.cam_pos_topic]:
            self.get_logger().error(
                "BAD CONFIG: output topic is the same as an input topic. "
                f"UNITY_POS_TOPIC={self.unity_pos_topic}, "
                f"CAM_POS_TOPIC={self.cam_pos_topic}, "
                f"ROBOTS_POS_TOPIC={self.robots_pos_topic}"
            )

    def unity_pos_callback(self, msg: PoseArray) -> None:
        poses = cast(Sequence[Pose], msg.poses)

        self.unity_count += 1
        self.have_unity = True
        self.unity_poses = deepcopy(list(poses))

        self.get_logger().info(
            f"Received Unity PoseArray #{self.unity_count}: "
            f"poses={len(self.unity_poses)}, "
            f"robot_ids={self.pose_ids_text(self.unity_poses)}"
        )

        self.publish_merged_poses(reason="new unity message")

    def cam_pos_callback(self, msg: PoseArray) -> None:
        poses = cast(Sequence[Pose], msg.poses)

        self.cam_count += 1
        self.have_cam = True
        self.cam_poses = deepcopy(list(poses))

        self.get_logger().info(
            f"Received cam PoseArray #{self.cam_count}: "
            f"poses={len(self.cam_poses)}, "
            f"robot_ids={self.pose_ids_text(self.cam_poses)}"
        )

        self.publish_merged_poses(reason="new cam message")

    def publish_merged_poses(self, reason: str) -> None:
        merged_poses: list[Pose] = []

        if self.have_unity:
            merged_poses.extend(self.sort_poses_by_robot_id(self.unity_poses))

        if self.have_cam:
            merged_poses.extend(self.sort_poses_by_robot_id(self.cam_poses))

        if not merged_poses:
            self.get_logger().warn("No poses available to publish.")
            return

        duplicate_ids = self.find_duplicate_ids(merged_poses)

        if duplicate_ids:
            self.get_logger().warn(
                "Duplicate robot IDs in merged output: "
                f"{duplicate_ids}. Because order is unity first, then cam, "
                "Game-master will effectively use the cam pose for duplicate IDs."
            )

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame_id
        msg.poses = merged_poses

        self.robots_pos_pub.publish(msg)

        self.publish_count += 1

        self.get_logger().info(
            f"Published merged PoseArray #{self.publish_count} because {reason}: "
            f"total_poses={len(merged_poses)}, "
            f"robot_ids={self.pose_ids_text(merged_poses)}, "
            f"have_unity={self.have_unity}, "
            f"have_cam={self.have_cam}"
        )

    def sort_poses_by_robot_id(self, poses: list[Pose]) -> list[Pose]:
        return sorted(poses, key=robot_id_from_pose)

    def pose_ids_text(self, poses: list[Pose]) -> list[int]:
        return [robot_id_from_pose(pose) for pose in poses]

    def find_duplicate_ids(self, poses: list[Pose]) -> list[int]:
        seen_ids: set[int] = set()
        duplicate_ids: set[int] = set()

        for pose in poses:
            robot_id = robot_id_from_pose(pose)

            if robot_id in seen_ids:
                duplicate_ids.add(robot_id)

            seen_ids.add(robot_id)

        return sorted(duplicate_ids)


def main() -> None:
    rclpy.init()

    node = RobotPositionMergerNode()

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