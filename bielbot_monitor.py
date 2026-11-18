
    
  
#!/usr/bin/env python3
"""Read-only Nav2 diagnostic monitor for manual-motion testing (ROS 2 Humble)."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class BielbotMonitor(Node):
    def __init__(self):
        super().__init__('bielbot_monitor')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('stamped_cmd', False)
        self.declare_parameter('path_tolerance', 0.15)
        self.declare_parameter('obstacle_warning_distance', 0.8)
        self.declare_parameter('data_timeout', 2.0)
        self.declare_parameter('print_period', 1.0)
        self.path = None
        self.scan = None
        self.cmd = None
        self.cmd_at = None
        self.scan_at = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path, '/planned_path', self.on_path, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        cmd_topic = self.get_parameter('cmd_topic').value
        cmd_type = TwistStamped if self.get_parameter('stamped_cmd').value else Twist
        self.create_subscription(cmd_type, cmd_topic, self.on_cmd, 10)
        self.create_timer(float(self.get_parameter('print_period').value), self.report)
        self.get_logger().info('Diagnostic en lecture seule: /planned_path, /scan, TF, ' + cmd_topic)

    def on_path(self, msg):
        self.path = msg if msg.poses else None
        self.get_logger().info(f'PATH: {len(msg.poses)} poses, frame={msg.header.frame_id}')

    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = time.monotonic()

    def on_cmd(self, msg):
        self.cmd = msg.twist if hasattr(msg, 'twist') else msg
        self.cmd_at = time.monotonic()

    def report(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('data_timeout').value)
        if self.path is None:
            self.get_logger().warn('WAITING: aucun chemin /planned_path')
            return
        frame = self.path.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(frame, 'base_link', Time(), timeout=Duration(seconds=0.1))
            x, y = tf.transform.translation.x, tf.transform.translation.y
            points = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
            

	    
            if len(points) == 1:
                deviation = math.hypot(x-points[0][0], y-points[0][1])
            else:
                deviation = min(point_segment_distance(x, y, *a, *b) for a, b in zip(points[:-1], points[1:]))
            following = deviation <= float(self.get_parameter('path_tolerance').value)
        except TransformException as exc:
            self.get_logger().warn(f'TF INDISPONIBLE: {exc}')
            return

        scan_fresh = self.scan is not None and self.scan_at is not None and now-self.scan_at < timeout
        nearest = None
        if scan_fresh:
            valid = [r for r in self.scan.ranges if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max]
            nearest = min(valid) if valid else None
        obstacle = nearest is not None and nearest < float(self.get_parameter('obstacle_warning_distance').value)
        cmd_fresh = self.cmd is not None and self.cmd_at is not None and now-self.cmd_at < timeout
        if cmd_fresh:
            vx, vy, wz = self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z
            velocity = f'vx={vx:+.3f} m/s | vy={vy:+.3f} m/s | wz={wz:+.3f} rad/s'
            moving_command = abs(vx)>0.01 or abs(vy)>0.01 or abs(wz)>0.02
        else:
            velocity = 'INDISPONIBLE / PERIMEE'
            moving_command = False

        state = 'FOLLOWING (proche du chemin)' if following else 'OFF PATH (ecart au chemin)'
        if obstacle:
            state += ' | OBSTACLE PROCHE (pas preuve de contournement)'
        if cmd_fresh and not moving_command:
            state += ' | COMMANDE QUASI NULLE'
        if not cmd_fresh:
            state += ' | CMD NON RECUE'
        scan_text = f'{nearest:.2f} m' if nearest is not None else ('aucune portee valide' if scan_fresh else 'SCAN INDISPONIBLE / PERIME')
        self.get_logger().info(f'[{state}] ecart={deviation:.3f} m | obstacle_min={scan_text} | cmd: {velocity}')


def main():
    rclpy.init()
    node = BielbotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


