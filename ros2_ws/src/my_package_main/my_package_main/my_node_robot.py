import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray
import sys
import numpy as np

from .task_type import TaskType
from .robot_state import RobotState
from .map import Map
from .theta_star import ThetaStar
from .orca import ORCA
from .pose import Pose


ROBOT_RADIUS = 91 # mm (noté r)
ROBOT_MAX_SPEED = 10 # mm/s (vitesse max robot holonome)
TASK_DONE_TIMEOUT = 5 # s Temps max avant le passage en waiting après un tache terminé
LOOP_PERIOD = 0.1 # s Période de la boule principal loop


def pos_to_cell(pos: tuple)->tuple:
    """
    Renvoi l'indice de la cellule de la carte corespondant
    à la position donnée.
    Pre:
        Couple des coordonnées de la position sur la carte.
    Post:
        Couple des indices de la cellule.
    """
    return int(pos[0]/Map.SIZE), int(pos[1]/Map.SIZE)

def cell_to_pos(cell: tuple)->tuple:
    """
    Renvoi la position sur la carte de la cellule donnée.
    Pre:
        Couple des indices de la cellule.
    Post:
        Couple des coordonnées de la position sur la carte.
    """
    return (cell[0]+0.5)*Map.SIZE, (cell[1]+0.5)*Map.SIZE


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

        self.waypoints = []

        self.task_done_timeout = 0

        # -- Init sub et pub --

        # Subscriber task
        self.task_sub = self.create_subscription(Float32MultiArray,"/robot_task", self.listener_task, 10)
        
        # Subscriber pose
        self.pose_sub = self.create_subscription(Float32MultiArray,"/robot_pose", self.listener_pose, 10)

        # Publisher state
        self.state_pub = self.create_publisher(Int8MultiArray, "/robot_state", 10)
        
        # -- Main loop --
        self.timer = self.create_timer(LOOP_PERIOD, self.loop)
        
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

        if self.state == RobotState.SEARCHING_PACKET:
            if not self.__is_at_pos(self.packet_pickup_pos):
                # Robot pas encore à la position de récupération, déplacement

                if len(self.waypoints) == 0:
                    # Pas de trajéctoire généré, génération
                    self.__generate_waypoints_to(self.packet_pickup_pos)
                # Déplacement
                self.__follow_waypoints()

            else:
                # Robot à la position de récupération, déplacement du packet
                self.set_state(RobotState.TRANSPORTING)


        elif self.state == RobotState.TRANSPORTING:
            if not self.__is_at_pos(self.packet_dropoff_pos):
                # Robot pas encore à la position de livraison, déplacement

                if len(self.waypoints) == 0:
                    # Pas de trajéctoire généré, génération
                    self.__generate_waypoints_to(self.packet_dropoff_pos)
                # Déplacement
                self.__follow_waypoints()

            else:
                # Robot à la position de livraison, validation de la tache
                self.set_state(RobotState.TASK_DONE)
                self.task_done_timeout = TASK_DONE_TIMEOUT

        elif self.state == RobotState.TASK_DONE:
            self.task_done_timeout -= LOOP_PERIOD

            # Si timeout, passage en waiting
            if self.task_done_timeout <= 0:
                self.set_state(RobotState.WAITING)

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

    def __is_at_pos(self, pos)->bool:
        """
        Test si le robot est arrivé à une position donnée.
        Pre :
            Couple des coordonnées de la position sur la carte.
        Post :
            Un booléen vrai si le robot est à la position donnée.
        """
        # Utilisation des coordonnées du robot holonome
        dist = np.sqrt((self.pose.X - pos[0])**2 + (self.pose.Y - pos[1])**2)
        return dist < ROBOT_RADIUS

    def __generate_waypoints_to(self, pos):
        """
        Calcul les points de passages pour aller à une position donnée.
        Pre:
            Couple des coordonnées de la position sur la carte.
        """
        robot_pos = (self.pose.X, self.pose.Y)
        if self.theta_star.has_line_of_sight(pos_to_cell(robot_pos), pos_to_cell(pos)):
            self.waypoints = [pos]
        else:
            waypoints = self.theta_star.compute_waypoints(pos_to_cell(robot_pos), pos_to_cell(pos))
            for i in range(len(waypoints)):
                waypoints[i] = cell_to_pos(waypoints[i])
            self.waypoints = waypoints

    def __follow_waypoints(self):
        """
        Déplace le robot avec ORCA pour suivre les points de passages.
        """
        # Retour si aucun trajet planifié
        if len(self.waypoints) == 0:
            return

        if self.__is_at_pos(self.waypoints[0]):
            # Suppression point de passage courant si on y est
            self.waypoints.pop()

            # Retour si point final atteint
            if len(self.waypoints) == 0:
                return

        robot_pos = (self.pose.X, self.pose.Y)

        # Regénération points de passages si point de passage suivant invisible
        if not self.theta_star.has_line_of_sight(pos_to_cell(robot_pos), pos_to_cell(self.waypoints[0])):
            self.__generate_waypoints_to(pos_to_cell(self.waypoints[-1]))

        # Récupération de la vitesse qui nous mène au point
        speed_pref = self.__get_pref_speed(self.waypoints[0])

        # ORCA
        speed = self.orca.compute_optimal_velocity(speed_pref)

        # Actionnement des moteurs
        # TODO, appliquer la vitesse au moteurs

    def __get_pref_speed(self, pos)->tuple:
        """
        Calcul la vitesse préféré pour aller vers un point donnée.
        Pre:
            Couple des coordonnées de la position sur la carte.
        Post:
            Couple des coordonnées de la vitesse calculé.
        """
        robot_pos = (self.pose.X, self.pose.Y)
        speed_pref = (pos[0]-robot_pos[0], pos[1]-robot_pos[1])

        # TODO, gestion de l'amplitude
        # Ici vitesse max tout le temps
        norm = np.sqrt(speed_pref[0]**2 + speed_pref[1]**2)
        speed_pref = (speed_pref[0]*ROBOT_MAX_SPEED/norm, speed_pref[1]*ROBOT_MAX_SPEED/norm)

        return speed_pref

def main(args=None):
    robot_id = int(sys.argv[1]) if len(sys.argv) > 1 else "None"
    
    rclpy.init(args=args)
    node = RobotNode(robot_id)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
