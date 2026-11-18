
#!/usr/bin/env python3

import socket
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


REVPI_IP = "192.168.50.2"
PORT = 5002

FREQUENCY = 1.0 #20.0 is the real MPPI freq, 1 is monitirs freq
CMD_TIMEOUT = 0.5


class MPPIUDPBridge(Node):

    def __init__(self):
        super().__init__('mppi_udp_bridge')

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd_time = None

        # Adapter si /cmd_vel est un TwistStamped
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / FREQUENCY,
            self.send_udp
        )

        self.get_logger().info(
            'MPPI UDP Bridge started'
        )

    def cmd_callback(self, msg):

        self.vx = msg.linear.x
        self.wz = msg.angular.z

        self.last_cmd_time = time.monotonic()

    def send_udp(self):

        now = time.monotonic()

        # Watchdog : pas de commande recente
        if (
            self.last_cmd_time is None
            or now - self.last_cmd_time > CMD_TIMEOUT
        ):
            vx = 0.0
            wz = 0.0
            status = "TIMEOUT"
        else:
            vx = self.vx
            wz = self.wz
            status = "MPPI"

        # Même format que le script de test
        payload = struct.pack("!ff", vx, wz)

        try:
            self.sock.sendto(
                payload,
                (REVPI_IP, PORT)
            )
        except OSError as exc:
            self.get_logger().error(
                f'UDP send failed: {exc}'
            )
            return

        self.get_logger().info(
            f'[{status}] '
            f'vx={vx:+.3f} m/s | '
            f'wz={wz:+.3f} rad/s'
        )

    def stop(self):
        try:
            self.sock.sendto(
                struct.pack("!ff", 0.0, 0.0),
                (REVPI_IP, PORT)
            )
        finally:
            self.sock.close()


def main():

    rclpy.init()

    node = MPPIUDPBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

