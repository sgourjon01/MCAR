import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray
import sys
import numpy as np

from .task_type import TaskType
from .robot_state import RobotState
from .theta_star import ThetaStar
from .orca import ORCA
from .pose import Pose


ROBOT_RADIUS = 91 # mm (noté r)
ROBOT_MAX_SPEED = 10 # mm/s (vitesse max robot holonome)


class RobotNode(Node):
    """
    Classe qui gère un robot.
    """

    def __init__(self, robot_id: int):
        super().__init__("Robot_"+str(robot_id))
        self.robot_id = robot_id
        self.name = "Robot_"+str(robot_id)

        # -- Variables du robot --

        self.theta_star = ThetaStar()
        self.orca = ORCA(self.robot_id, ROBOT_RADIUS, ROBOT_MAX_SPEED)

        self.state = RobotState.WAITING
        self.pose = Pose(ROBOT_RADIUS)

        self.packet_pickup_pos = (0, 0)
        self.packet_dropoff_pos = (0, 0)

        # -- Init sub et pub --

        # Subscriber task
        self.task_sub = self.create_subscription(Float32MultiArray,"/robot_task", self.listener_task, 10)
        
        # Subscriber pose
        self.pose_sub = self.create_subscription(Float32MultiArray,"/robot_pose", self.listener_pose, 10)

        # Publisher state
        self.state_pub = self.create_publisher(Int8MultiArray, "/robot_state", 10)
        
        # -- Main loop --
        loop_timer_period = 0.1  # secondes
        self.timer = self.create_timer(loop_timer_period, self.loop)
        
        self.get_logger().info("Lancement du noeud pour le robot "+self.name+".")

    def listener_task(self, msg):
        """
        Subscriber task
        Traitement des ordes du Manager.
        """

        if len(msg.data) < 2:
            # Message incomplet
            return

        robot_id = msg.data[0]
        task = msg.data[1]

        # Vérification ordre pour nous
        if robot_id != self.robot_id:
            return

        # MOVE_PACKET
        if task == TaskType.MOVE_PACKET:
            # Si on est en attente, on accepte l'ordre
            if self.state == RobotState.WAITING:
                self.packet_pickup_pos = (msg.data[2], msg.data[3])
                self.packet_dropoff_pos = (msg.data[4], msg.data[5])

                self.set_state(RobotState.SEARCHING_PACKET)

            # Ordre déjà en cours
            else:
                self.get_logger().info("Un ordre a été reçu alors qu'une tâche est déjà en cours. Ordre ignoré.")

        # ACKNOWLEDGE
        if task == TaskType.ACK:
            # Si on à finit une tache, passer en waiting
            if self.state == RobotState.TASK_DONE:
                self.set_state(RobotState.WAITING)

    def listener_pose(self, msg):
        """
        Subscriber pose
        Actualisation des variables du robot.
        """

        if len(msg.data) != 7:
            # Message non conforme
            return

        robot_id = msg.data[0]

        pose = Pose(ROBOT_RADIUS)
        pose.set_real(
            msg.data[1],  # x
            msg.data[2],  # y
            msg.data[3],  # theta
            np.sqrt(msg.data[4] ** 2 + msg.data[5] ** 2),  # V
            msg.data[6]  # theta_dot
        )

        if robot_id == self.robot_id:
            # Mise à jour pose si c'est la pose du robot
            self.pose.set(pose)

        # Mise à jour robots sur ORCA
        self.orca.update_agent(robot_id, (pose.X, pose.Y), (pose.X_dot, pose.Y_dot))

    def loop(self):
        """
        Main loop
        Logique principale
        """

        # TODO
        pass

    def set_state(self, state):
        """
        Change l'état du robot.
        """
        # Changement de l'état
        self.state = state

        # Notification du changement sur le topic robot_state
        self.__notify_state_change()

    def __notify_state_change(self):
        """
        Publie sur le topic robot_state le changement d'etat.
        """
        msg = Int8MultiArray()
        msg.data = [self.robot_id, self.state]
        self.state_pub.publish(msg)


def main(args=None):
    robot_id = int(sys.argv[1]) if len(sys.argv) > 1 else "None"
    
    rclpy.init(args=args)
    node = RobotNode(robot_id)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
