import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray
import sys
import numpy as np

from .robot_state import RobotState
from .theta_star import ThetaStar
from .orca import ORCA
from .pose import Pose


ROBOT_RADIUS = 91 * 1e-3 # 91 mm (r)


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
        self.orca = ORCA(self.robot_id)

        self.__state = RobotState.WAITING

        self.pose = Pose(ROBOT_RADIUS)

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

        if len(msg.data) <= 0:
            # Message vide
            return
        
        # TODO

    def listener_pose(self, msg):
        """
        Subscriber pose
        Actualisation des variables du robot.
        """

        if len(msg.data) != 7:
            # Message non conforme
            return

        id = msg.data[0]

        pose = Pose(ROBOT_RADIUS)
        pose.set_real(
            msg.data[1],  # x
            msg.data[2],  # y
            msg.data[3],  # theta
            np.sqrt(msg.data[4] ** 2 + msg.data[5] ** 2),  # V
            msg.data[6]  # theta_dot
        )

        if id == self.robot_id:
            # Mise à jour pose si c'est la pose du robot
            self.pose.set(pose)

        # TODO mise à jour pour ORCA
    
    def loop(self):
        """
        Main loop
        Logique principale
        """

        # TODO
        pass
    

def main(args=None):
    robot_id = int(sys.argv[1]) if len(sys.argv) > 1 else "None"
    
    rclpy.init(args=args)
    node = RobotNode(robot_id)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
