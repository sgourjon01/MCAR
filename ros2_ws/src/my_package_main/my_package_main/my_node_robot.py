import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray
import sys
import numpy as np

from .robot_state import RobotState
from .theta_star import ThetaStar


class RobotNode(Node):
    """
    Classe qui gère un robot.
    """

    def __init__(self, robot_id: int):
        super().__init__("Robot_"+str(robot_id))
        self.robot_id = robot_id
        self.name = "Robot_"+str(robot_id)
        
        # -- Constantes du robot --
        self.r = 91 * 1e-3 # 91 mm

        # -- Variables du robot --

        self.theta_star = ThetaStar()

        self.__state = RobotState.WAITING

        self.__pos_l = np.zeros(2)
        self.__pos_a = 0.0
        self.__vel_l = np.zeros(2)
        self.__vel_a = 0.0

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

        if id == self.robot_id:
            # Mise a jour pose si c'est la pose du robot
            self.__pos_l[0] = msg.data[1]
            self.__pos_l[1] = msg.data[2]
            self.__pos_a = msg.data[3]
            self.__vel_l[0] = msg.data[4]
            self.__vel_l[1] = msg.data[5]
            self.__vel_a = msg.data[6]

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
