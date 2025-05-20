import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import zmq


class ZMQToJoy(Node):
    def __init__(self):
        super().__init__('zmq_to_joy')
        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('bind_port', 5555)
        ip = self.get_parameter('bind_ip').get_parameter_value().string_value
        port = self.get_parameter('bind_port').get_parameter_value().integer_value

        ctx = zmq.Context.instance()
        self.socket = ctx.socket(zmq.PULL)
        self.socket.bind(f'tcp://{ip}:{port}')

        self.publisher = self.create_publisher(Joy, 'joy_out', 10)
        self.timer = self.create_timer(0.01, self.poll)

    def poll(self):
        try:
            data = self.socket.recv_pyobj(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        msg = Joy()
        msg.axes = data.get('axes', [])
        msg.buttons = data.get('buttons', [])
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZMQToJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
