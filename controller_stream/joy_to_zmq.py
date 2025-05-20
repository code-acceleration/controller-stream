import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import zmq


class JoyToZMQ(Node):
    def __init__(self):
        super().__init__('joy_to_zmq')
        self.declare_parameter('target_ip', '127.0.0.1')
        self.declare_parameter('target_port', 5555)
        ip = self.get_parameter('target_ip').get_parameter_value().string_value
        port = self.get_parameter('target_port').get_parameter_value().integer_value

        ctx = zmq.Context.instance()
        self.socket = ctx.socket(zmq.PUSH)
        self.socket.connect(f'tcp://{ip}:{port}')

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

    def joy_callback(self, msg: Joy):
        data = {
            'axes': list(msg.axes),
            'buttons': list(msg.buttons)
        }
        try:
            self.socket.send_pyobj(data, flags=zmq.NOBLOCK)
        except zmq.Again:
            self.get_logger().warning('ZMQ send would block, message dropped')


def main(args=None):
    rclpy.init(args=args)
    node = JoyToZMQ()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
