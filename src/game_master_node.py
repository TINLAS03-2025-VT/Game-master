#!/usr/bin/env python3
"""
Game-master ROS 2 node.

This node implements a small state machine for a tag game with robots.

States:
- WAIT:      wait for robots to announce that they are ready
- RUNNING:   pick one runner, send start command, check caught/seen logic
- PAUSE:     game is paused
- POST_GAME: game is finished, show winner

Keyboard controls when running interactively:
- Enter: start from WAIT, or return from POST_GAME to WAIT
- Space: pause/resume
- r: reset at any time
- q: quit process

ROS topics:
- Publishes /game_master/counter       std_msgs/msg/UInt64
- Publishes /game_master/heartbeat     std_msgs/msg/String
- Publishes /game/command              std_msgs/msg/String
- Publishes /robots/seen               std_msgs/msg/Int32
- Subscribes /game_master/input        std_msgs/msg/String
- Subscribes /game/robots/ready        std_msgs/msg/Int32
- Subscribes /robots/pos               geometry_msgs/msg/PoseArray

Command examples on /game_master/input:
- start
- reset
- pause
- resume
- status
- seen 1
"""

import argparse
import math
import os
import random
import select
import shlex
import sys
import termios
import threading
import time
import tty
from enum import Enum
from queue import SimpleQueue
from typing import NoReturn, Sequence, cast

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String, UInt64, Bool


class GameState(Enum):
    WAIT = "wait"
    RUNNING = "running"
    PAUSE = "pause"
    POST_GAME = "post_game"


class RosCommandArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> NoReturn:
        raise ValueError(message)

    def exit(self, status: int = 0, message: str | None = None) -> NoReturn:
        if message:
            raise ValueError(message)
        raise ValueError(f"argparse exited with status {status}")


def make_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def distance_between_poses(first: Pose, second: Pose) -> float:
    dx = first.position.x - second.position.x
    dy = first.position.y - second.position.y
    return math.sqrt((dx * dx) + (dy * dy))


def normalize_angle_degrees(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def yaw_from_pose_degrees(pose: Pose) -> float:
    q = pose.orientation

    siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
    cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))

    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def angle_from_hunter_to_runner_degrees(hunter_pose: Pose, runner_pose: Pose) -> float:
    dx = runner_pose.position.x - hunter_pose.position.x
    dy = runner_pose.position.y - hunter_pose.position.y

    return math.degrees(math.atan2(dy, dx))


def hunter_angle_error_degrees(hunter_pose: Pose, runner_pose: Pose) -> float:
    hunter_yaw = yaw_from_pose_degrees(hunter_pose)
    angle_to_runner = angle_from_hunter_to_runner_degrees(hunter_pose, runner_pose)

    return normalize_angle_degrees(angle_to_runner - hunter_yaw)


class GameMasterNode(Node):
    def __init__(self) -> None:
        node_name = os.getenv("NODE_NAME", "game_master")
        super().__init__(node_name)

        self.counter_topic = os.getenv("COUNTER_TOPIC", "/game_master/counter")
        self.heartbeat_topic = os.getenv("HEARTBEAT_TOPIC", "/game_master/heartbeat")
        self.input_topic = os.getenv("INPUT_TOPIC", "/game_master/input")
        self.robots_pos_topic = os.getenv("ROBOTS_POS_TOPIC", "/unity/pos")
        self.robots_ready_topic = os.getenv("ROBOTS_READY_TOPIC", "/game/robots/ready")
        self.game_command_topic = os.getenv("GAME_COMMAND_TOPIC", "/game/command")
        self.robots_seen_topic = os.getenv("ROBOTS_SEEN_TOPIC", "/robots/seen")

        self.timer_period_sec = float(os.getenv("TIMER_PERIOD_SEC", "0.2"))
        self.runner_win_seconds = float(os.getenv("RUNNER_WIN_SECONDS", "60.0"))
        self.caught_distance = float(os.getenv("CAUGHT_DISTANCE", "1"))
        self.seen_distance = float(os.getenv("SEEN_DISTANCE", "4.0"))
        self.lose_distance = float(os.getenv("LOSE_DISTANCE", "4.75"))
        self.seen_half_fov_degrees = float(os.getenv("SEEN_HALF_FOV_DEGREES", "10.0"))

        qos = make_qos()

        self.counter_pub = self.create_publisher(UInt64, self.counter_topic, qos)
        self.heartbeat_pub = self.create_publisher(String, self.heartbeat_topic, qos)
        self.game_command_pub = self.create_publisher(String, self.game_command_topic, qos)
        self.robots_seen_pub = self.create_publisher(Bool, self.robots_seen_topic, qos)

        self.input_sub = self.create_subscription(
            String,
            self.input_topic,
            self.game_command_callback,
            qos,
        )

        self.robots_ready_sub = self.create_subscription(
            Int32,
            self.robots_ready_topic,
            self.robots_ready_callback,
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
        self.ready_count = 0
        self.game_command_count = 0

        self.state = GameState.WAIT
        self.ready_robot_ids: set[int] = set()
        self.active_robot_ids: set[int] = set()
        self.robot_poses: dict[int, Pose] = {}

        self.runner_id: int | None = None
        self.caught_by_robot_id: int | None = None
        self.winner_text = ""

        self.last_seen = time.monotonic()
        self.runner_seen: bool = False

        self.active_running_seconds = 0.0
        self.last_loop_time = time.monotonic()

        self.command_queue: SimpleQueue[str] = SimpleQueue()
        self.shutdown_requested = threading.Event()
        self.keyboard_thread: threading.Thread | None = None
        self.game_command_parser = self.create_game_command_parser()

        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)

        self.start_keyboard_thread()

        self.get_logger().info("Game-master node started")
        self.get_logger().info(f"State: {self.state.value}")
        self.get_logger().info(f"Publishing counter:     {self.counter_topic} [std_msgs/msg/UInt64]")
        self.get_logger().info(f"Publishing heartbeat:   {self.heartbeat_topic} [std_msgs/msg/String]")
        self.get_logger().info(f"Publishing commands:    {self.game_command_topic} [std_msgs/msg/String]")
        self.get_logger().info(f"Publishing seen robots: {self.robots_seen_topic} [std_msgs/msg/Bool]")
        self.get_logger().info(f"Subscribing input:      {self.input_topic} [std_msgs/msg/String]")
        self.get_logger().info(f"Subscribing ready:      {self.robots_ready_topic} [std_msgs/msg/Int32]")
        self.get_logger().info(f"Subscribing robots:     {self.robots_pos_topic} [geometry_msgs/msg/PoseArray]")
        self.get_logger().info("Keyboard: Enter=start/begin, Space=pause/resume, r=reset, q=quit")

    def start_keyboard_thread(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn("No interactive terminal detected; keyboard controls disabled.")
            return

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            name="keyboard-input",
            daemon=True,
        )
        self.keyboard_thread.start()

    def keyboard_loop(self) -> None:
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin)

            while not self.shutdown_requested.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)

                if not readable:
                    continue

                key = sys.stdin.read(1)

                if key in ("\n", "\r"):
                    self.command_queue.put("start")
                elif key == " ":
                    if self.state == GameState.RUNNING:
                        self.command_queue.put("pause")
                    elif self.state == GameState.PAUSE:
                        self.command_queue.put("resume")
                elif key.lower() == "~":
                    self.command_queue.put("reset")
                elif key.lower() == "q":
                    self.get_logger().info("Quit requested from keyboard.")
                    self.shutdown_requested.set()
                    rclpy.shutdown()
                    return

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def timer_callback(self) -> None:
        self.process_queued_commands()
        self.run_state_machine_once()
        self.publish_counter_and_heartbeat()
        self.counter += 1

    def publish_counter_and_heartbeat(self) -> None:
        counter_msg = UInt64()
        counter_msg.data = self.counter
        self.counter_pub.publish(counter_msg)

        heartbeat_msg = String()
        heartbeat_msg.data = (
            f"alive state={self.state.value} "
            f"counter={self.counter} "
            f"ready={sorted(self.ready_robot_ids)} "
            f"active={sorted(self.active_robot_ids)} "
            f"runner={self.runner_id} "
            f"input_count={self.input_count} "
            f"posearray_count={self.posearray_count} "
            f"ready_count={self.ready_count} "
            f"game_command_count={self.game_command_count} "
            f"active_running_seconds={self.active_running_seconds:.1f}"
        )
        self.heartbeat_pub.publish(heartbeat_msg)

    def process_queued_commands(self) -> None:
        while not self.command_queue.empty():
            command = self.command_queue.get()
            self.handle_command_text(command)

    def run_state_machine_once(self) -> None:
        match self.state:
            case GameState.WAIT:
                return

            case GameState.RUNNING:
                self.running_loop_once()

            case GameState.PAUSE:
                return

            case GameState.POST_GAME:
                return

    def robots_ready_callback(self, msg: Int32) -> None:
        robot_id = int(msg.data)
        self.ready_count += 1

        if self.state != GameState.WAIT:
            self.get_logger().info(
                f"Ignoring ready robot {robot_id}; game is already in state {self.state.value}."
            )
            return

        if robot_id not in self.ready_robot_ids:
            self.ready_robot_ids.add(robot_id)
            self.print_ready_robots()
        else:
            self.get_logger().info(f"Robot {robot_id} is already marked ready.")

    def robots_pos_callback(self, msg: PoseArray) -> None:
        self.posearray_count += 1

        poses = cast(Sequence[Pose], msg.poses)

        current_robot_poses: dict[int, Pose] = {}

        for pose in poses:
            robot_id = int(round(pose.position.z))

            if robot_id in current_robot_poses:
                self.get_logger().warn(
                    f"Duplicate robot ID {robot_id} in {self.robots_pos_topic}; using latest pose."
                )

            current_robot_poses[robot_id] = pose

        self.robot_poses = current_robot_poses

        self.get_logger().debug(
            f"Received robots PoseArray #{self.posearray_count} "
            f"on {self.robots_pos_topic}: "
            f"poses={len(poses)}, "
            f"robot_ids={sorted(self.robot_poses.keys())}, "
            f"frame_id='{msg.header.frame_id}'"
        )

    def game_command_callback(self, msg: String) -> None:
        self.input_count += 1
        self.command_queue.put(msg.data)

    def create_game_command_parser(self) -> argparse.ArgumentParser:
        parser = RosCommandArgumentParser(
            prog="game_command",
            description="Parse commands received on /game_master/input",
        )

        subparsers = parser.add_subparsers(
            dest="command",
            required=True,
        )

        start_parser = subparsers.add_parser(
            "start",
            help="Start the game from the WAIT state",
        )
        start_parser.set_defaults(handler=self.handle_start_command)

        reset_parser = subparsers.add_parser(
            "reset",
            help="Reset the game at any time",
        )
        reset_parser.set_defaults(handler=self.handle_reset_command)

        pause_parser = subparsers.add_parser(
            "pause",
            help="Pause the game",
        )
        pause_parser.set_defaults(handler=self.handle_pause_command)

        resume_parser = subparsers.add_parser(
            "resume",
            help="Resume the game",
        )
        resume_parser.set_defaults(handler=self.handle_resume_command)

        status_parser = subparsers.add_parser(
            "status",
            help="Print current game status",
        )
        status_parser.set_defaults(handler=self.handle_status_command)

        seen_parser = subparsers.add_parser(
            "seen",
            help="Manually publish that a robot can see the runner",
        )
        seen_parser.add_argument(
            "seen",
            type=int,
            choices=[0, 1],
            help="1 if hunters can see the runner, 0 if not",
        )
        seen_parser.set_defaults(handler=self.handle_seen_command)

        return parser

    def handle_command_text(self, raw_command: str) -> None:
        self.game_command_count += 1
        raw_command = raw_command.strip()

        self.get_logger().info(
            f"Received game command #{self.game_command_count}: {raw_command}"
        )

        if not raw_command:
            self.get_logger().warn("Ignoring empty game command")
            return

        try:
            tokens = shlex.split(raw_command)
            args = self.game_command_parser.parse_args(tokens)
            args.handler(args)
        except ValueError as exc:
            self.get_logger().error(f"Invalid game command '{raw_command}': {exc}")
        except Exception as exc:
            self.get_logger().error(f"Failed to handle game command '{raw_command}': {exc}")

    def handle_start_command(self, args: argparse.Namespace) -> None:
        del args

        if self.state == GameState.WAIT:
            self.start_game()
        elif self.state == GameState.POST_GAME:
            self.get_logger().info("Returning from POST_GAME to WAIT.")
            self.reset_local_state(send_reset_to_robots=False)
        else:
            self.get_logger().warn(f"Cannot start from state {self.state.value}.")

    def handle_reset_command(self, args: argparse.Namespace) -> None:
        del args
        self.reset_game()

    def handle_pause_command(self, args: argparse.Namespace) -> None:
        del args
        self.pause_game()

    def handle_resume_command(self, args: argparse.Namespace) -> None:
        del args
        self.resume_game()

    def handle_status_command(self, args: argparse.Namespace) -> None:
        del args
        self.print_status()

    def handle_seen_command(self, args: argparse.Namespace) -> None:
        self.publish_seen_robot(bool(args.seen))

    def start_game(self) -> None:
        if not self.ready_robot_ids:
            self.get_logger().warn("Cannot start: no robots are ready.")
            return

        self.active_robot_ids = set(self.ready_robot_ids)
        self.runner_id = random.choice(sorted(self.active_robot_ids))
        self.caught_by_robot_id = None
        self.winner_text = ""
        self.active_running_seconds = 0.0
        self.last_loop_time = time.monotonic()
        self.state = GameState.RUNNING

        self.get_logger().info("======================================")
        self.get_logger().info("GAME STARTED")
        self.get_logger().info(f"Active robots: {sorted(self.active_robot_ids)}")
        self.get_logger().info(f"Runner: robot {self.runner_id}")
        self.get_logger().info("Hunters: " + str(sorted(self.get_hunter_ids())))
        self.get_logger().info("======================================")

        self.publish_game_command(f"start {self.runner_id}")

    def pause_game(self) -> None:
        if self.state != GameState.RUNNING:
            self.get_logger().warn(f"Cannot pause from state {self.state.value}.")
            return

        self.state = GameState.PAUSE
        self.publish_game_command("pause")
        self.get_logger().info("Game paused.")

    def resume_game(self) -> None:
        if self.state != GameState.PAUSE:
            self.get_logger().warn(f"Cannot resume from state {self.state.value}.")
            return

        self.state = GameState.RUNNING
        self.last_loop_time = time.monotonic()
        self.publish_game_command("resume")
        self.get_logger().info("Game resumed.")

    def reset_game(self) -> None:
        self.publish_game_command("reset")
        self.reset_local_state(send_reset_to_robots=False)

    def reset_local_state(self, send_reset_to_robots: bool) -> None:
        if send_reset_to_robots:
            self.publish_game_command("reset")

        self.state = GameState.WAIT
        self.ready_robot_ids.clear()
        self.active_robot_ids.clear()
        self.robot_poses.clear()
        self.runner_id = None
        self.caught_by_robot_id = None
        self.winner_text = ""
        self.active_running_seconds = 0.0
        self.last_loop_time = time.monotonic()

        self.runner_seen = False
        self.last_seen = time.monotonic()

        self.get_logger().info("Game reset. State is WAIT.")
        self.print_ready_robots()

    def running_loop_once(self) -> None:
        now = time.monotonic()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        self.active_running_seconds += dt

        if self.runner_id is None:
            self.get_logger().error("RUNNING state has no runner_id. Resetting.")
            self.reset_game()
            return

        if self.active_running_seconds >= self.runner_win_seconds:
            self.enter_post_game(
                winner_text=f"Runner robot {self.runner_id} won by surviving for {self.runner_win_seconds:.1f} seconds."
            )
            return

        runner_pose = self.robot_poses.get(self.runner_id)

        if runner_pose is None:
            self.get_logger().warn(f"Waiting for runner pose. runner_id={self.runner_id}")
            return

        caught_by = self.find_hunter_that_caught_runner(runner_pose)

        if caught_by is not None:
            self.caught_by_robot_id = caught_by
            self.enter_post_game(
                winner_text=f"Hunter robot {caught_by} won by catching runner robot {self.runner_id}."
            )
            return

        can_see = self.can_hunters_see_runner(runner_pose)        
        if can_see != self.runner_seen:
            self.runner_seen = can_see
            self.publish_seen_robot(self.runner_seen)


    def find_hunter_that_caught_runner(self, runner_pose: Pose) -> int | None:
        for hunter_id in self.get_hunter_ids():
            hunter_pose = self.robot_poses.get(hunter_id)

            if hunter_pose is None:
                continue

            distance = distance_between_poses(hunter_pose, runner_pose)

            if distance <= self.caught_distance:
                self.get_logger().info(
                    f"Runner caught: hunter={hunter_id}, runner={self.runner_id}, distance={distance:.3f}"
                )
                return hunter_id

        return None

    def can_hunters_see_runner(self, runner_pose: Pose) -> bool:
        hunter_ids = sorted(self.get_hunter_ids())
        known_pose_ids = sorted(self.robot_poses.keys())

        runner_yaw = yaw_from_pose_degrees(runner_pose)

        self.get_logger().info(
            "========== SEEN CHECK =========="
        )
        self.get_logger().info(
            f"runner_id={self.runner_id}, "
            f"runner_seen_before={self.runner_seen}, "
            f"runner_pos=({runner_pose.position.x:.3f}, {runner_pose.position.y:.3f}), "
            f"runner_z_id={runner_pose.position.z:.1f}, "
            f"runner_yaw={runner_yaw:.2f} deg"
        )
        self.get_logger().info(
            f"hunters={hunter_ids}, known_pose_ids={known_pose_ids}, "
            f"seen_distance={self.seen_distance:.3f}, "
            f"lose_distance={self.lose_distance:.3f}, "
            f"seen_half_fov_degrees={self.seen_half_fov_degrees:.3f}"
        )

        for hunter_id in hunter_ids:
            hunter_pose = self.robot_poses.get(hunter_id)

            if hunter_pose is None:
                self.get_logger().info(
                    f"hunter={hunter_id}: NO POSE, skipping"
                )
                continue

            distance = distance_between_poses(hunter_pose, runner_pose)

            hunter_yaw = yaw_from_pose_degrees(hunter_pose)
            angle_to_runner = angle_from_hunter_to_runner_degrees(hunter_pose, runner_pose)
            angle_error = hunter_angle_error_degrees(hunter_pose, runner_pose)

            self.get_logger().info(
                f"hunter={hunter_id}: "
                f"pos=({hunter_pose.position.x:.3f}, {hunter_pose.position.y:.3f}), "
                f"z_id={hunter_pose.position.z:.1f}, "
                f"yaw={hunter_yaw:.2f} deg, "
                f"angle_to_runner={angle_to_runner:.2f} deg, "
                f"angle_error={angle_error:.2f} deg, "
                f"distance={distance:.3f}"
            )

            if self.runner_seen:
                self.get_logger().info(
                    f"hunter={hunter_id}: already-seen mode, "
                    f"angle ignored, checking distance <= lose_distance "
                    f"({distance:.3f} <= {self.lose_distance:.3f})"
                )

                if distance <= self.lose_distance:
                    self.get_logger().info(
                        f"hunter={hunter_id}: STILL SEES runner because distance is close enough"
                    )
                    self.get_logger().info(
                        "========== SEEN CHECK RESULT: TRUE =========="
                    )
                    return True

                self.get_logger().info(
                    f"hunter={hunter_id}: too far to keep seeing runner"
                )
                continue

            self.get_logger().info(
                f"hunter={hunter_id}: not-seen-yet mode, "
                f"checking distance and angle"
            )

            if distance > self.seen_distance:
                self.get_logger().info(
                    f"hunter={hunter_id}: CANNOT SEE runner, "
                    f"distance too far ({distance:.3f} > {self.seen_distance:.3f})"
                )
                continue

            if abs(angle_error) > self.seen_half_fov_degrees:
                self.get_logger().info(
                    f"hunter={hunter_id}: CANNOT SEE runner, "
                    f"angle outside FOV "
                    f"(|{angle_error:.3f}| > {self.seen_half_fov_degrees:.3f})"
                )
                continue

            self.get_logger().info(
                f"hunter={hunter_id}: SEES runner, "
                f"distance={distance:.3f}, angle_error={angle_error:.3f}"
            )
            self.get_logger().info(
                "========== SEEN CHECK RESULT: TRUE =========="
            )
            return True

        self.get_logger().info(
            "No hunter sees runner."
        )
        self.get_logger().info(
            "========== SEEN CHECK RESULT: FALSE =========="
        )
        return False

    def enter_post_game(self, winner_text: str) -> None:
        self.state = GameState.POST_GAME
        self.winner_text = winner_text

        self.get_logger().info("======================================")
        self.get_logger().info("GAME OVER")
        self.get_logger().info(winner_text)
        self.get_logger().info("Press Enter to return to WAIT.")
        self.get_logger().info("======================================")

        self.publish_game_command("reset")

    def get_hunter_ids(self) -> list[int]:
        if self.runner_id is None:
            return sorted(self.active_robot_ids)

        return sorted(robot_id for robot_id in self.active_robot_ids if robot_id != self.runner_id)

    def publish_game_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.game_command_pub.publish(msg)
        self.get_logger().info(f"Published game command on {self.game_command_topic}: {command}")

    def publish_seen_robot(self, seen:bool) -> None:
        msg = Bool()
        msg.data = seen
        self.robots_seen_pub.publish(msg)
        self.get_logger().info(f"Published seen runner on {self.robots_seen_topic}: {seen}")

    def print_ready_robots(self) -> None:
        self.get_logger().info("Ready robots: " + str(sorted(self.ready_robot_ids)))

    def print_status(self) -> None:
        self.get_logger().info("======================================")
        self.get_logger().info(f"State: {self.state.value}")
        self.get_logger().info(f"Ready robots: {sorted(self.ready_robot_ids)}")
        self.get_logger().info(f"Active robots: {sorted(self.active_robot_ids)}")
        self.get_logger().info(f"Runner: {self.runner_id}")
        self.get_logger().info(f"Hunters: {self.get_hunter_ids()}")
        self.get_logger().info(f"Known robot poses: {sorted(self.robot_poses.keys())}")
        self.get_logger().info(f"Active running seconds: {self.active_running_seconds:.1f}")
        self.get_logger().info(f"Winner: {self.winner_text}")
        self.get_logger().info("======================================")


def main() -> None:
    rclpy.init()

    node = GameMasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.shutdown_requested.set()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()