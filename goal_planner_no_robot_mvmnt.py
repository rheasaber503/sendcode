
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer, TransformListener, TransformException


class GoalPlanner(Node):

    def __init__(self):
        super().__init__('goal_planner')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose'
        )

        self.path_pub = self.create_publisher(
            Path, '/planned_path', 10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        self.get_logger().info(
            'Goal Planner prêt. Choisis une destination dans RViz.'
        )

    def goal_callback(self, goal_pose):
        if goal_pose.header.frame_id != 'map':
            self.get_logger().error(
                'La destination doit être exprimée dans le frame map.'
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except TransformException as ex:
            self.get_logger().error(f'TF indisponible : {ex}')
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()

        start.pose.position.x = transform.transform.translation.x
        start.pose.position.y = transform.transform.translation.y
        start.pose.position.z = transform.transform.translation.z
        start.pose.orientation = transform.transform.rotation

        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal_pose
        request.planner_id = 'GridBased'
        request.use_start = True

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Planner Server indisponible.')
            return

        self.get_logger().info('Calcul du chemin A*...')
        future = self.client.send_goal_async(request)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Demande de chemin refusée.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        response = future.result()

        if response.status != 4:  # STATUS_SUCCEEDED
            self.get_logger().error(
                f'Calcul échoué. Statut : {response.status}'
            )
            return

        path = response.result.path

        self.path_pub.publish(path)

        self.get_logger().info(
            f'Chemin A* calculé : {len(path.poses)} points.'
        )


def main():
    rclpy.init()
    node = GoalPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
