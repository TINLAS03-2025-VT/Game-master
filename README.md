# Game-master

ROS 2 Humble Python node that runs in Docker and joins the robot ROS network through a WireGuard sidecar container.

This node is **not a relay**.

Default behavior:

- publishes `/game_master/counter` as `std_msgs/msg/UInt64`
- publishes `/game_master/heartbeat` as `std_msgs/msg/String`
- subscribes to `/game_master/input` as `std_msgs/msg/String`
- subscribes to `/robots/pos` as `geometry_msgs/msg/PoseArray`
