# Game Master

ROS 2 Humble service that controls the Jachtseizoen game state. It registers ready robots, starts a round, selects the runner, handles pause/resume/reset commands, evaluates runner visibility, and ends the round when the runner is caught or survives the configured time limit.

## Features

- Central game state machine with `WAIT`, `RUNNING`, `PAUSE`, and `POST_GAME` states.
- Random runner selection from the ready robot list.
- Start, pause, resume, reset, status, and manual visibility commands.
- Automatic hunter win and runner win detection.
- Runner visibility calculation based on distance and field-of-view thresholds.
- Robot Position Merger service for combining Unity and camera pose streams into one world state.
- Docker-based deployment through WireGuard and CycloneDDS.

## Repository contents

| Path | Purpose |
|---|---|
| `src/game_master_node.py` | Main Game Master ROS 2 node. |
| `src/robot_position_merger_node.py` | Merges Unity and camera PoseArray streams into the shared robot position stream. |
| `compose.yaml` | Standalone Game Master deployment with WireGuard client and Robot Position Merger. |
| `Dockerfile` | Container image definition for the Game Master services. |
| `cyclonedds/client.xml` | CycloneDDS client configuration for the WireGuard interface. |
| `requirements.txt` | Python dependencies. |

## Getting started

### Requirements

- Docker and Docker Compose
- A WireGuard peer configuration for the Jacht server
- Access to the ROS 2 server network

### Install

1. Clone the repository.
2. Place the WireGuard peer configuration in the expected local path:

```bash
mkdir -p wireguard-client/wg_confs
cp wg0.conf wireguard-client/wg_confs/wg0.conf
```

3. Start the services:

```bash
docker compose up -d
```

4. Check that the containers are running:

```bash
docker compose ps
```

5. Follow the logs:

```bash
docker compose logs -f game-master
```

The normal project deployment starts the Game Master and Robot Position Merger from the server stack. This standalone compose file is useful when the Game Master is deployed as a separate WireGuard client.

## Configuration

### Game Master

| Variable | Default / current value | Effect |
|---|---:|---|
| `NODE_NAME` | `game_master` | ROS node name. |
| `COUNTER_TOPIC` | `/game_master/counter` | Counter output topic. |
| `HEARTBEAT_TOPIC` | `/game_master/heartbeat` | Human-readable status output topic. |
| `INPUT_TOPIC` | `/game_master/input` | Remote operator command input. |
| `ROBOTS_POS_TOPIC` | `/robots/pos` | Shared robot position input. |
| `ROBOTS_READY_TOPIC` | `/game/robots/ready` | Robot ready input. |
| `GAME_COMMAND_TOPIC` | `/game/command` | Game command output to robots and Unity. |
| `ROBOTS_SEEN_TOPIC` | `/robots/seen` | Runner visibility output. |
| `TIMER_PERIOD_SEC` | `0.01` | Main Game Master timer period. |
| `RUNNER_WIN_SECONDS` | `300.0` | Time after which the runner wins. |
| `CAUGHT_DISTANCE` | `1.2` | Maximum distance for a hunter catch check. |
| `CAUGHT_HALF_FOV_DEGREES` | `45.0` | Required catch field-of-view half angle. |
| `SEEN_DISTANCE` | `3.0` | Distance threshold for initial runner visibility. |
| `LOSE_DISTANCE` | `3.5` | Distance threshold for keeping the runner visible after it was seen. |
| `SEEN_HALF_FOV_DEGREES` | `20.0` | Required visibility field-of-view half angle. |

### Robot Position Merger

| Variable | Default / current value | Effect |
|---|---:|---|
| `NODE_NAME` | `robot_position_merger` | ROS node name. |
| `UNITY_POS_TOPIC` | `/unity/pos` | Unity position input. |
| `CAM_POS_TOPIC` | `/cam/pos` | Camera/tracker position input. |
| `ROBOTS_POS_TOPIC` | `/robots/pos` | Merged position output. |
| `OUTPUT_FRAME_ID` | `map` | Frame ID used in merged PoseArray messages. |
| `PUBLISH_RATE_HZ` | `60.0` source default, `10.0` in standalone compose | Output publish rate. |
| `POSE_TIMEOUT_SEC` | `2.0` | Removes stale Unity/camera poses after this time. |

## Commands and actions

### Keyboard controls

Keyboard controls are active when the container has an interactive terminal.

| Key | Action |
|---|---|
| `Enter` | Start from `WAIT`, or return from `POST_GAME` to `WAIT`. |
| `Space` | Pause while running, resume while paused. |
| `~` | Reset the game. |
| `q` | Quit the Game Master process. |

### Remote commands

Remote commands are sent as `std_msgs/msg/String` messages to the configured input topic.

| Command | Effect |
|---|---|
| `start` | Start a new round from `WAIT`. |
| `reset` | Reset the game state and publish `reset` to robots. |
| `pause` | Pause an active round. |
| `resume` | Resume a paused round. |
| `status` | Print the current game status in the logs. |
| `seen 0` | Publish runner visibility as false manually. |
| `seen 1` | Publish runner visibility as true manually. |

Example:

```bash
ros2 topic pub --once /game_master/input std_msgs/msg/String "{data: 'status'}"
```

## Connections

| Direction | Interface | Purpose |
|---|---|---|
| Incoming | `/robots/pos` (`geometry_msgs/msg/PoseArray`) | Current merged positions of all known robots. |
| Incoming | `/game/robots/ready` (`std_msgs/msg/Int32`) | Robot ready announcements. |
| Incoming | `/game_master/input` (`std_msgs/msg/String`) | Remote operator commands. |
| Outgoing | `/game/command` (`std_msgs/msg/String`) | Lifecycle commands for physical and simulated robots. |
| Outgoing | `/robots/seen` (`std_msgs/msg/Bool`) | Whether hunters currently see the runner. |
| Outgoing | `/game_master/counter` (`std_msgs/msg/UInt64`) | Liveness counter. |
| Outgoing | `/game_master/heartbeat` (`std_msgs/msg/String`) | Human-readable runtime state. |
| Network | WireGuard + CycloneDDS | Joins the project ROS 2 network. |
