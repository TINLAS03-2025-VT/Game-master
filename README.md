# ROS Python Sidecar Boilerplate

Boilerplate ROS 2 Humble Python node that runs in Docker and joins the ROS network through a WireGuard sidecar container.

Default behavior:

- subscribes to `/cam/pos` (`geometry_msgs/msg/PoseArray`)
- republishes to `/robots/pos` (`geometry_msgs/msg/PoseArray`)
- publishes heartbeat on `/boilerplate/heartbeat` (`std_msgs/msg/String`)

## Files

```text
src/ros_boilerplate_node.py      Main subscriber/publisher node
src/ros_self_test.py             Test publisher node
Dockerfile                       ROS Humble image
compose.remote.yaml              WireGuard sidecar + app container
compose.remote.test.yaml         WireGuard sidecar + test publisher
compose.local.yaml               Local no-WireGuard test
cyclonedds/client.xml            CycloneDDS config using wg0
.github/workflows/docker-image.yml
```

## WireGuard config

Create this local-only folder:

```bash
mkdir -p wireguard-client/wg_confs
nano wireguard-client/wg_confs/wg0.conf
```

Paste the WireGuard peer config from the server maintainer.

Do not commit `wireguard-client/`.

## Local test without WireGuard

```bash
docker compose -f compose.local.yaml up --build
```

In another terminal:

```bash
docker run --rm -it --net=host ros:humble-ros-base bash
source /opt/ros/humble/setup.bash
ros2 topic list -t
ros2 topic echo /boilerplate/heartbeat
```

## Remote test over WireGuard

```bash
docker compose -f compose.remote.test.yaml pull
docker compose -f compose.remote.test.yaml up
```

On the ROS server:

```bash
ros2 topic echo /cam/pos
```

## Run the node over WireGuard

```bash
docker compose -f compose.remote.yaml pull
docker compose -f compose.remote.yaml up -d
docker logs -f ros-python-sidecar-boilerplate
```

Stop:

```bash
docker compose -f compose.remote.yaml down --remove-orphans
```

## Environment variables

```text
NODE_NAME=ros_python_boilerplate
SUB_TOPIC=/cam/pos
PUB_TOPIC=/robots/pos
HEARTBEAT_TOPIC=/boilerplate/heartbeat
RELIABLE_QOS=true
TIMER_PERIOD_SEC=1.0
```

## GitHub Container Registry image

Default image name:

```text
ghcr.io/tinlas03-2025-vt/ros-python-sidecar-boilerplate:latest
```

Change this in these files if your repo name is different:

- `.github/workflows/docker-image.yml`
- `compose.remote.yaml`
- `compose.remote.test.yaml`
- `compose.local.yaml`
