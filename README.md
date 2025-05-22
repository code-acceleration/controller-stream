# controller_stream

This package provides two ROS2 nodes that bridge `sensor_msgs/Joy` messages over ZeroMQ.
Both nodes are implemented in C++ and exchange compact binary payloads for
minimal bandwidth and latency.

## Building

```bash
colcon build --packages-select controller_stream
source install/setup.bash
```

## Usage

### Forward ROS Joy messages to ZMQ

```bash
ros2 run controller_stream joy_to_zmq --ros-args -p target_ip:=<ip> -p target_port:=5555
```
The node keeps only the latest message queued to avoid delays.

### Convert ZMQ messages back to ROS Joy

```bash
ros2 run controller_stream zmq_to_joy --ros-args -p bind_ip:=0.0.0.0 -p bind_port:=5555
```
The receiver drops old data if it falls behind, ensuring low latency.

Replace `<ip>` with the remote address that should receive the ZMQ packets.
