# ros2-zmq-bridge

This package provides two ROS2 nodes that bridge `sensor_msgs/Joy` messages over ZeroMQ.
Both nodes are implemented in C++ and exchange compact binary payloads for
minimal bandwidth and latency.

## Building

```bash
colcon build --packages-select ros2-zmq-bridge
source install/setup.bash
```

If you previously built the package under the old name `ros2-zmq-briadge`,
remove the `build` and `install` directories in your workspace before
rebuilding. Otherwise ROS may warn about missing paths.

## Usage

### Forward ROS Joy messages to ZMQ

```bash
ros2 run ros2-zmq-bridge joy_to_zmq --ros-args -p target_ip:=<ip> -p target_port:=5555
```
The node keeps only the latest message queued to avoid delays.

### Convert ZMQ messages back to ROS Joy

```bash
ros2 run ros2-zmq-bridge zmq_to_joy --ros-args -p bind_ip:=0.0.0.0 -p bind_port:=5555
```
The receiver drops old data if it falls behind, ensuring low latency.

Replace `<ip>` with the remote address that should receive the ZMQ packets.

### General ROS2/ZMQ Bridge

Use the `ros2_zmq_bridge` node for arbitrary message types. Set the `mode` parameter to either `ros2_to_zmq` or `zmq_to_ros2` and provide the `type`, `topic`, `ip` and `port` parameters.

```bash
ros2 run ros2-zmq-bridge ros2_zmq_bridge --ros-args \
  -p mode:=ros2_to_zmq -p type:=std_msgs/msg/String \
  -p topic:=chatter -p ip:=127.0.0.1 -p port:=5555
```
