import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray
import sys
import numpy as np

from .robot_state import RobotState
from .theta_star import ThetaStar
from .orca import ORCA
from .kinematics import Kinematics

# Les topics échangent des mm et mm/s ; le nœud travaille en unités SI (m, m/s).
MM_TO_M = 1e-3


class RobotNode(Node):
    """
    Classe qui gère un robot.
    """

    def __init__(self, robot_id: int):
        super().__init__("Robot_"+str(robot_id))
        self.robot_id = robot_id
        self.name = "Robot_"+str(robot_id)

        # -- Constantes du robot (SI) --
        self.r = 91 * 1e-3  # Rayon du robot : 91 mm
        self.wheel_base = 0.182  # TODO : A MESURER - distance entre les roues [m]
        self.max_wheel_speed = 0.3  # TODO : A MESURER - vitesse max d'une roue [m/s]
        self.effective_offset = self.r  # D = r (article de Snape), soit R = 2r

        # -- Variables du robot --
        # Instanciation des classes Kinematics, ThetaStar et ORCA
        self.kinematics = Kinematics(
            wheel_base=self.wheel_base,
            effective_offset=self.effective_offset,
            robot_radius=self.r,
            max_wheel_speed=self.max_wheel_speed,
        )
        self.theta_star = ThetaStar()
        # TODO : ORCA doit raisonner sur le centre effectif. Lui fournir
        # self.kinematics.effective_radius (rayon) et
        # self.kinematics.max_effective_speed (vitesse max).
        self.orca = ORCA(self.robot_id)

        self.__state = RobotState.WAITING

        self.__pos_l = np.zeros(2)  # Position du centre q [m]
        self.__pos_a = 0.0  # Orientation [rad]
        self.__vel_l = np.zeros(2)  # Vitesse linéaire du centre q [m/s]
        self.__vel_a = 0.0  # Vitesse angulaire [rad/s]

        # Etat du centre effectif (position [m], vitesse [m/s]) de chaque robot,
        # indexé par identifiant. Entrée prévue pour ORCA.
        self.__effective_states = {}

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
        Actualisation des variables du robot et des états effectifs pour ORCA.
        """

        if len(msg.data) != 7:
            # Message non conforme
            return

        id = int(msg.data[0])

        # Conversion mm -> m (positions et vitesses linéaires uniquement)
        position = np.array([msg.data[1], msg.data[2]]) * MM_TO_M
        theta = float(msg.data[3])
        velocity = np.array([msg.data[4], msg.data[5]]) * MM_TO_M
        omega = float(msg.data[6])

        if id == self.robot_id:
            # Mise a jour pose si c'est la pose du robot
            self.__pos_l = position
            self.__pos_a = theta
            self.__vel_l = velocity
            self.__vel_a = omega

        # Robot réel -> centre effectif (holonome), pour soi et pour les voisins
        self.__effective_states[id] = (
            self.kinematics.effective_center(position, theta),
            self.kinematics.effective_velocity(theta, velocity, omega),
        )

        # TODO mise à jour pour ORCA (à partir de self.__effective_states)

    def loop(self):
        """
        Main loop
        Logique principale
        """

        if self.__state == RobotState.WAITING:
            self.send_wheel_command(0.0, 0.0)
            return

        # Vitesse souhaitée pour le centre effectif (holonome)
        effective_velocity = self.compute_effective_velocity()

        # Centre effectif -> consigne des roues (non holonome)
        v_left, v_right = self.kinematics.wheel_speeds(self.__pos_a, effective_velocity)

        self.send_wheel_command(v_left, v_right)

    def compute_effective_velocity(self) -> np.ndarray:
        """
        Calcule la vitesse du centre effectif à appliquer.

        Chaîne prévue : Theta* (waypoint) -> vitesse préférée dirigée du
        centre effectif vers le waypoint -> ORCA -> vitesse retenue.
        """

        # TODO : Theta* + ORCA. Arrêt en attendant.
        return np.zeros(2)

    def send_wheel_command(self, v_left: float, v_right: float):
        """
        Envoie la consigne de vitesse aux roues [m/s].
        """

        # TODO : définir le canal vers les moteurs (pilotage direct ou nouveau
        # topic, à documenter dans Topics.md).
        self.get_logger().debug(f"Consigne roues : g={v_left:.3f} d={v_right:.3f} m/s")


def main(args=None):
    robot_id = int(sys.argv[1]) if len(sys.argv) > 1 else "None"

    rclpy.init(args=args)
    node = RobotNode(robot_id)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()