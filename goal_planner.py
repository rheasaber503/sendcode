
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException


class GoalPlanner(Node):

    def __init__(self):
        super().__init__('goal_planner') # crée un nœud ROS 2 nommé /goal_planner.

        # Seuils de recalcul  au moins 10 cm , 10 degavant un recalcul, au moins 2 secondes entre deux demandes de calcul.
        self.distance_threshold = 0.10
        self.angle_threshold = math.radians(10.0)
        self.min_replan_interval = 2.0

        # TF: listener reçoit les transformations TF et les conserve temporairement dans tf_buffer.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # Client d'action Nav2: C'est la connexion vers le Planner Server. Notre node est client et Nav2 serveur
        self.client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        # Publication du chemin: Ce publisher envoie des messages nav_msgs/Path: une succession de poses qui forment le chemin.
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # Destination RViz: Chaque fois qu'une destination arrive sur /goal_pose, ROS 2 appelle automatiquement goal_callback(msg). RViz publie un message PoseStamped sur /goal_pose.
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Carte SLAM: chaque fois que SLAM publie une carte, ROS 2 appelle map_callback(msg)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.current_goal = None
        self.last_planned_pose = None
        self.last_plan_time = None

        self.map_changed = False
        self.planning = False
        self.goal_version = 0

        # Surveillance toutes les 0,5 seconde: ROS 2 appellera check_replanning() environ toutes les 0,5 seconde, tant que le nœud fonctionne.
        self.timer = self.create_timer(
            0.5,
            self.check_replanning
        )

        self.get_logger().info(
            'Goal Planner prêt : recalcul automatique activé.'
        )

    def goal_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().error(
                'La destination doit être dans le frame map.'
            )
            return

        self.current_goal = msg
        self.goal_version += 1
        self.last_planned_pose = None #force un recalcul au prochain passage du timer.
        self.map_changed = True

        self.get_logger().info(
            'Nouvelle destination reçue.'
        )

    def map_callback(self, msg):
        # Une nouvelle carte SLAM a été publiée.
        # Elle peut contenir des changements.
        pass  #self.map_changed = True

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.1)
            )

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation

            return pose

        except TransformException as ex:
            self.get_logger().warn(
                f'TF indisponible : {ex}'
            )
            return None

    @staticmethod
    def yaw(pose):
        q = pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

    def robot_moved(self, pose): #ne sera plus utilié et donc on ne regardera plusangle_threshold = math.radians(10.0) et distance_threshold =0.1
        if self.last_planned_pose is None:
            return True

        previous = self.last_planned_pose

        dx = pose.pose.position.x - previous.pose.position.x
        dy = pose.pose.position.y - previous.pose.position.y

        distance = math.hypot(dx, dy)

        angle = math.atan2(
            math.sin(self.yaw(pose) - self.yaw(previous)),
            math.cos(self.yaw(pose) - self.yaw(previous))
        )

        return (
            distance >= self.distance_threshold
            or abs(angle) >= self.angle_threshold
        )

    def check_replanning(self): #orchestre:est appelée toutes les 0,5 seconde par le timer.
        if self.current_goal is None or self.planning: #Si aucune destination n'a été choisie, ou si un calcul est déjà en cours, on ne fait rien.
            return

        pose = self.get_robot_pose() #récupère la position actuelle. Si TF est indisponible, on attend.
        if pose is None:
            return

        now = self.get_clock().now()

        if self.last_plan_time is not None:#  évite de lancer des calculs à moins de deux secondes d'intervalle.
            elapsed = (
                now - self.last_plan_time
            ).nanoseconds / 1e9

            if elapsed < self.min_replan_interval:
                return

       
            #self.robot_moved(pose) # vérifie (si à cette pose il a bougé par rapport à last planned pose) si le robot a suffisamment bougé ou si SLAM a publié une carte: dans ces deux cas il y a demande de calcul d'un autre path or
        if not self.map_changed:
        
            return

        if not self.client.server_is_ready():
            self.get_logger().warn(
                'Planner Server indisponible.'
            )
            return

        self.send_plan_request(pose) #Si Nav2 est prêt, on demande un nouveau chemin A* à Nav2

    def send_plan_request(self, start): #start=pose actuelle  qui devient last_planned_pose
        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = self.current_goal
        request.planner_id = 'GridBased'
        request.use_start = True

        self.planning = True
        self.last_plan_time = self.get_clock().now()
        self.last_planned_pose = start #reset to last planned_pose 
        self.map_changed = False #reset to false

        version = self.goal_version

        self.get_logger().info(
            'Recalcul du chemin A*...'
        )

        future = self.client.send_goal_async(request) # envoie la demande anev tous les requests set
        future.add_done_callback(
            lambda f: self.goal_response_callback(f, version)
        )

    def goal_response_callback(self, future, version):
        try:
            handle = future.result()

            if handle is None or not handle.accepted:
                self.get_logger().error(
                    'Demande de chemin refusée.'
                )
                self.planning = False
                return

            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.result_callback(f, version)
            )

        except Exception as ex:
            self.get_logger().error(str(ex))
            self.planning = False

    def result_callback(self, future, version):
        self.planning = False

        try:
            response = future.result()

            # Ignorer les résultats d'une ancienne destination
            if version != self.goal_version:
                return

            if response.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f'Échec du calcul : statut {response.status}'
                )
                self.map_changed = True
                return

            path = response.result.path
            self.path_pub.publish(path)

            self.get_logger().info(
                f'Chemin actualisé : {len(path.poses)} points.'
            )

        except Exception as ex:
            self.get_logger().error(
                f'Erreur lors du résultat : {ex}'
            )
            self.map_changed = True


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
