#!/usr/bin/env python3

import os
import time
from copy import deepcopy
from dataclasses import dataclass
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

@dataclass
class StoredPose:
    pose: Pose
    last_update_sec: float


class RobotPositionMergerNode(Node):
    def __init__(self) -> None:
        super().__init__(os.getenv("NODE_NAME", "robot_position_merger"))

        self.unity_pos_topic = os.getenv("UNITY_POS_TOPIC", "/unity/pos")
        self.cam_pos_topic = os.getenv("CAM_POS_TOPIC", "/cam/pos")
        self.robots_pos_topic = os.getenv("ROBOTS_POS_TOPIC", "/robots/pos")
        self.output_frame_id = os.getenv("OUTPUT_FRAME_ID", "map")

        self.publish_rate_hz = float(os.getenv("PUBLISH_RATE_HZ", "60.0"))
        self.pose_timeout_sec = float(os.getenv("POSE_TIMEOUT_SEC", "2.0"))

        if self.publish_rate_hz <= 0.0:
            raise ValueError("PUBLISH_RATE_HZ must be greater than 0")

        if self.pose_timeout_sec <= 0.0:
            raise ValueError("POSE_TIMEOUT_SEC must be greater than 0")

        self.publish_period_sec = 1.0 / self.publish_rate_hz

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

        self.unity_poses_by_id: dict[int, StoredPose] = {}
        self.cam_poses_by_id: dict[int, StoredPose] = {}

        self.publish_count = 0
        self.unity_count = 0
        self.cam_count = 0
        self.last_published_robot_ids: set[int] = set()

        self.publish_timer = self.create_timer(
            self.publish_period_sec,
            self.publish_timer_callback,
        )

        self.get_logger().info("Robot position merger started")
        self.get_logger().info(f"Subscribing Unity poses: {self.unity_pos_topic}")
        self.get_logger().info(f"Subscribing cam poses:   {self.cam_pos_topic}")
        self.get_logger().info(f"Publishing merged poses: {self.robots_pos_topic}")
        self.get_logger().info("Merge priority: Unity first, cam overrides duplicate robot IDs")
        self.get_logger().info(f"Publish rate: {self.publish_rate_hz:.1f} Hz")
        self.get_logger().info(f"Pose timeout: {self.pose_timeout_sec:.1f} seconds")

        if self.robots_pos_topic in [self.unity_pos_topic, self.cam_pos_topic]:
            self.get_logger().error(
                "BAD CONFIG: output topic is the same as an input topic. "
                f"UNITY_POS_TOPIC={self.unity_pos_topic}, "
                f"CAM_POS_TOPIC={self.cam_pos_topic}, "
                f"ROBOTS_POS_TOPIC={self.robots_pos_topic}"
            )

    def unity_pos_callback(self, msg: PoseArray) -> None:
        self.unity_count += 1

        updated_ids = self.update_source_poses(
            source_poses=self.unity_poses_by_id,
            msg=msg,
            source_name="Unity",
        )

        self.get_logger().debug(
            f"Received Unity PoseArray #{self.unity_count}: "
            f"poses={len(updated_ids)}, "
            f"robot_ids={updated_ids}"
        )


    def cam_pos_callback(self, msg: PoseArray) -> None:
        self.cam_count += 1

        updated_ids = self.update_source_poses(
            source_poses=self.cam_poses_by_id,
            msg=msg,
            source_name="cam",
        )

        self.get_logger().debug(
            f"Received cam PoseArray #{self.cam_count}: "
            f"poses={len(updated_ids)}, "
            f"robot_ids={updated_ids}"
        )


    def update_source_poses(
        self,
        source_poses: dict[int, StoredPose],
        msg: PoseArray,
        source_name: str,
    ) -> list[int]:
        poses = cast(Sequence[Pose], msg.poses)
        now_sec = time.monotonic()

        updated_ids: list[int] = []

        for pose in poses:
            robot_id = robot_id_from_pose(pose)

            if robot_id in source_poses:
                self.get_logger().debug(
                    f"Updating existing {source_name} pose for robot {robot_id}"
                )

            source_poses[robot_id] = StoredPose(
                pose=deepcopy(pose),
                last_update_sec=now_sec,
            )

            updated_ids.append(robot_id)

        return updated_ids

    def publish_timer_callback(self) -> None:
        self.remove_stale_poses()
        self.publish_merged_poses()


    def remove_stale_poses(self) -> None:
        now_sec = time.monotonic()

        self.remove_stale_poses_from_source(
            source_poses=self.unity_poses_by_id,
            now_sec=now_sec,
            source_name="Unity",
        )

        self.remove_stale_poses_from_source(
            source_poses=self.cam_poses_by_id,
            now_sec=now_sec,
            source_name="cam",
        )


    def remove_stale_poses_from_source(
        self,
        source_poses: dict[int, StoredPose],
        now_sec: float,
        source_name: str,
    ) -> None:
        stale_robot_ids: list[int] = []

        for robot_id, stored_pose in list(source_poses.items()):
            age_sec = now_sec - stored_pose.last_update_sec

            if age_sec > self.pose_timeout_sec:
                stale_robot_ids.append(robot_id)
                del source_poses[robot_id]

        if stale_robot_ids:
            self.get_logger().info(
                f"Removed stale {source_name} poses after "
                f"{self.pose_timeout_sec:.1f}s timeout: "
                f"robot_ids={sorted(stale_robot_ids)}"
            )


    def publish_merged_poses(self) -> None:
        merged_poses_by_id: dict[int, Pose] = {}

        # Unity first
        for robot_id, stored_pose in self.unity_poses_by_id.items():
            merged_poses_by_id[robot_id] = deepcopy(stored_pose.pose)

        # Cam overwrites duplicate IDs
        duplicate_ids = sorted(
            set(self.unity_poses_by_id.keys()) & set(self.cam_poses_by_id.keys())
        )

        for robot_id, stored_pose in self.cam_poses_by_id.items():
            merged_poses_by_id[robot_id] = deepcopy(stored_pose.pose)

        current_robot_ids = set(merged_poses_by_id.keys())

        merged_poses = [
            merged_poses_by_id[robot_id]
            for robot_id in sorted(current_robot_ids)
        ]

        # If no robots are valid anymore, publish one empty PoseArray only once.
        # This clears Game-master's stored robot poses without spamming empty messages.
        if not merged_poses:
            if self.last_published_robot_ids:
                msg = PoseArray()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.output_frame_id
                msg.poses = []

                self.robots_pos_pub.publish(msg)
                self.publish_count += 1
                self.last_published_robot_ids = set()

                self.get_logger().info(
                    "Published one empty PoseArray because all poses timed out. "
                    "Further empty publishes will be skipped until new poses arrive."
                )

            else:
                self.get_logger().debug(
                    "No valid poses available and empty state already published; "
                    "skipping /robots/pos publish."
                )

            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame_id
        msg.poses = merged_poses

        self.robots_pos_pub.publish(msg)

        self.publish_count += 1
        self.last_published_robot_ids = current_robot_ids

        log_every_n_publishes = max(1, int(self.publish_rate_hz * 5.0))

        if self.publish_count == 1 or self.publish_count % log_every_n_publishes == 0:
            self.get_logger().info(
                f"Published merged PoseArray #{self.publish_count}: "
                f"total_poses={len(merged_poses)}, "
                f"robot_ids={self.pose_ids_text(merged_poses)}, "
                f"unity_ids={sorted(self.unity_poses_by_id.keys())}, "
                f"cam_ids={sorted(self.cam_poses_by_id.keys())}, "
                f"duplicate_ids={duplicate_ids}"
            )
        else:
            self.get_logger().debug(
                f"Published merged PoseArray #{self.publish_count}: "
                f"total_poses={len(merged_poses)}, "
                f"robot_ids={self.pose_ids_text(merged_poses)}"
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