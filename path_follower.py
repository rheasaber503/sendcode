#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        self.active_handle = None
        self.pending_path = None
        self.busy = False
        self.generation = 0
        self.last_feedback_time = 0.0
        self.cancel_requested = False

        self.get_logger().info(
            'Path Follower prêt. En attente de /planned_path.'
        )

    def path_callback(self, path):
        if not path.poses: #Vérifier que le chemin n'est pas vide
            self.get_logger().warn('Chemin vide ignoré.')
            return

        if path.header.frame_id != 'map': #impose que le chemin soit exprimé dans le repère map.
            self.get_logger().error(
                'Chemin refusé : frame différent de map.'
            )
            return

        distance = 0.0

        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position

            distance += math.hypot(
                p2.x - p1.x,
                p2.y - p1.y
            )

        self.get_logger().info(
            f'Longueur totale du chemin testee : {distance:.2f} m'
        )

        self.pending_path = path #conserve le nouveau chemin dans une variable et donne qd le robot est occupé avec un ancien path

        self.get_logger().info(
            f'Nouveau chemin reçu : {len(path.poses)} poses.'
        )

        if not self.busy:#script n'est pas déjà occupé par une opération d'envoi ou d'annulation,
            self.process_pending_path() 

    def process_pending_path(self):
        if self.pending_path is None or self.busy:
            return

        if not self.client.server_is_ready():
            self.get_logger().warn(
                'Controller Server indisponible.'
            )
            return

        self.busy = True # commence

        if self.active_handle is not None: #avant:chemin deja actif on le cancel et prepare pour donner le nouveau.apres: si chemin en ntrain d etre traite annulation slt si cancel request
        
            if self.cancel_requested:
                self.busy = False
                return
                
            self.cancel_requested = True
                
            self.get_logger().info(
                'demande Annulation de l ancien chemin...'
            )

            future = self.active_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_callback)
        else:
            self.send_pending_path() #auncun chemin deja actif on send le pending path

    def cancel_callback(self, future):
        try:
            response = future.result()

            if not response.goals_canceling:
                self.get_logger().error(
                    'Annulation non confirmée.'
                )
                self.busy = False
                return

            self.get_logger().info(
                'Annulation acceptée, attente de la fin.'
            )

        except Exception as ex:
            self.get_logger().error(
                f'Erreur annulation : {ex}'
            )
            self.busy = False

    def send_pending_path(self):
        if self.pending_path is None:
            self.busy = False
            return

        path = self.pending_path
        self.pending_path = None

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'#plugin qui vérifie si la destination est atteinte

        self.generation += 1
        generation = self.generation

        self.get_logger().info(
            'Envoi du chemin au Controller MPPI.'
        )

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            lambda f: self.goal_response_callback(
                f, generation
            )
        )

    def goal_response_callback(self, future, generation):
        try:
            handle = future.result()

            if handle is None or not handle.accepted:
                self.get_logger().error(
                    'FollowPath a refusé le chemin.'
                )
                self.busy = False
                return

            self.active_handle = handle
            self.busy = False

            self.get_logger().info(
                'Chemin accepté par MPPI.'
            )

            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.result_callback(
                    f, generation, handle
                )
            )

            if self.pending_path is not None:
                self.process_pending_path()

        except Exception as ex:
            self.get_logger().error(str(ex))
            self.busy = False

    def feedback_callback(self, msg):#afficher la distance restante
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_feedback_time < 1.0:#limite les logs à un par seconde, même si le Controller envoie des feedbacks (distance retante) plus fréquemment. distace restante par secondes. commeca c plus easy
            return

        self.last_feedback_time = now

        feedback = msg.feedback

        self.get_logger().info(
            f'Distance restante : '
            f'{feedback.distance_to_goal:.2f} m'
            f'Vitesse : {feedback.speed:.2f} m/s'
        )

    def result_callback(self, future, generation, handle):
        try:
            response = future.result()

            if self.active_handle is not handle:
                return

            self.active_handle = None
            self.cancel_requested = False

            if response.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    'Destination atteinte.'
                )

            elif response.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(
                    'Ancien chemin annulé.'
                )

            else:
                self.get_logger().error(
                    f'Suivi terminé avec statut {response.status}.'
		    f'Résultat : {response.result}'
                )

            self.busy = False

            if self.pending_path is not None:
                self.process_pending_path()

        except Exception as ex:
            self.get_logger().error(str(ex))
            self.busy = False

    def retry_callback(self):
        if self.pending_path is not None and not self.busy:
            self.process_pending_path()


def main():
    rclpy.init()
    node = PathFollower()

    node.create_timer(1.0, node.retry_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

