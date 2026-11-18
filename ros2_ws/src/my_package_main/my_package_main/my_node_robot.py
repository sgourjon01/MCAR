"""
    Thomas Eyer 14/11/2026
    
    Nœud Robot : suit une liste de waypoints (x, y) avec un robot différentiel.
    Chaîne : waypoint -> vitesse préférée -> vitesse effective v
         -> vitesses de roues u -> PWM -> trame série.
         
    Utilisation :   ros2 run <paquet> robot <id 1..4> par exemple
                    ros2 run <paquet> robot 1 &
                    ros2 run <paquet> robot 2 &
                    ros2 run <paquet> robot 3 &
                    ros2 run <paquet> robot 4 &
                (lances des processsus et noeuds séparés)
Unités SI (m, m/s, rad, s). robot_pose arrive en mm : conversion à la réception.
"""
import os
import sys

import numpy as np
import rclpy
import serial 
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray

# --- Constantes. TEMP = provisoire, à mesurer ---
MM_TO_M = 1e-3                # constante de conversion des mm vers les m
D_M = 0.091                   # distance q -> p. Vaut L/2 : M et M_inv ne sont valables que pour D = L/2
V_WHEEL_MAX = 0.30            # TEMP : vitesse max par roue [m/s]
V_MAX_EFF = V_WHEEL_MAX / np.sqrt(2)  # rayon du disque de vitesses atteignables quel que soit θ
V_PREF = 0.15                 # TEMP : doit rester < V_MAX_EFF pour laisser de la marge à ORCA
SLOW_DOWN_RADIUS_M = 0.15     # ralentissement avant le dernier waypoint
GOAL_TOLERANCE_M = 0.04       # TEMP : distance de validation d'un waypoint, mesurée sur p
POSE_TIMEOUT_S = 0.5          # pose plus vieille que ça -> arrêt moteurs
K_PWM = 5.0 / 0.035           # TEMP : PWM par m/s, déduit de motor.py (5 / diamètre roue)
PWM_MAX = 100
LOOP_PERIOD_S = 0.1


class RobotNode(Node):
    """
    Classe qui gère un robot.
    """

    def __init__(self, robot_id):
        super().__init__(f"Robot{robot_id}")
        self.robot_id = robot_id

        # État mesuré : centre q. État dérivé : centre effectif p et sa vitesse v.
        self.q = np.zeros(2)
        self.theta = 0.0
        self.omega = 0.0
        self.p = np.zeros(2)
        self.v = np.zeros(2)
        self.last_pose_time = None   # None tant qu'aucune pose n'est reçue

        # Objectifs intermédiaires
        self.waypoints = []
        self.waypoint_index = 0
        # TEMP : parcours de test, à remplacer par un robot_task reçu du manager.
        self.set_waypoints([(0.4, 0.3), (0.8, 0.3), (0.8, 0.7)])  # TEMP

        # Liaison série
        port_com = os.environ["SMARTIES_PORT_COM"]
        self.serial = serial.Serial(port_com, 115200, timeout=0.1)

        # Subscriber pose (topic partagé par tous les robots)
        self.pose_sub = self.create_subscription(
            Float32MultiArray, "robot_pose", self.listener_pose, 10)

        # Main loop
        self.timer = self.create_timer(LOOP_PERIOD_S, self.loop)

        self.get_logger().info(f"Lancement du noeud pour le robot {robot_id}")

    # ------------------------------------------------------------------
    # Capteur
    # ------------------------------------------------------------------
    def listener_pose(self, msg):
        """
        Subscriber pose
        Actualisation des variables du robot.
        """
        data = msg.data

        # stop si pas assez de donnée transmise ou si l'ID du robot ne correspond pas.
        if len(data) != 7 or int(round(data[0])) != self.robot_id:
            return
        # data : [id, x_mm, y_mm, θ_rad, vx_mm/s, vy_mm/s, ω_rad/s]
        self.q = np.array([data[1], data[2]]) * MM_TO_M
        self.theta = data[3]
        v_q = np.array([data[4], data[5]]) * MM_TO_M
        self.omega = data[6]

        # Centre effectif p = q + D*h, et sa vitesse : v = v_q + ω*D*h_perp.
        heading = np.array([np.cos(self.theta), np.sin(self.theta)])
        normal = np.array([-heading[1], heading[0]])
        self.p = self.q + D_M * heading
        self.v = v_q + self.omega * D_M * normal

        # Mémorise l'heure de perception, pour voir la pose est prérimée.
        self.last_pose_time = self.get_clock().now()

    def pose_is_fresh(self):
        """Faux si aucune pose reçue ou si la dernière est trop ancienne (< POSE_TIMEOUT_S)."""
        if self.last_pose_time is None:
            return False
        age_s = (self.get_clock().now() - self.last_pose_time).nanoseconds * 1e-9
        return age_s < POSE_TIMEOUT_S

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def set_waypoints(self, points):
        """Remplace la liste d'objectifs [(x, y), ...] en m. Point d'entrée du futur robot_task."""
        self.waypoints = [np.array(pt, dtype=float) for pt in points]
        self.waypoint_index = 0

    def update_waypoint(self):
        """Passe au waypoint suivant quand le centre effectif p est assez proche."""
        if self.waypoint_index >= len(self.waypoints):
            return

        target = self.waypoints[self.waypoint_index]
        if np.linalg.norm(target - self.p) > GOAL_TOLERANCE_M:
            return

        self.get_logger().info(f"Waypoint {self.waypoint_index} atteint")
        self.waypoint_index += 1
        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().info("Parcours terminé")

    def compute_preferred_velocity(self):
        """Vitesse de p vers le waypoint courant, sans tenir compte des autres robots."""
        if self.waypoint_index >= len(self.waypoints):
            return np.zeros(2)

        to_goal = self.waypoints[self.waypoint_index] - self.p
        dist = np.linalg.norm(to_goal)   # > 0 : update_waypoint a validé les points trop proches

        speed = V_PREF
        # On ne ralentit qu'au dernier point : les intermédiaires sont traversés sans s'arrêter.
        is_last = self.waypoint_index == len(self.waypoints) - 1
        if is_last and dist < SLOW_DOWN_RADIUS_M:
            speed = V_PREF * dist / SLOW_DOWN_RADIUS_M

        return speed * to_goal / dist

    # ------------------------------------------------------------------
    # Cinématique
    # ------------------------------------------------------------------
    def M(self, theta):
        """
        Renvoi la matrice M.
        Permet de passer entre les vitesses des
        roues u = (droite, gauche) et la vitesse effective v.
        """
        c = np.cos(theta)
        s = np.sin(theta)

        M = 1/2 * np.array([
            [c-s, c+s],
            [c+s, s-c]
        ])

        return M

    def M_inv(self, theta):
        """
        Renvoi la matrice M inverse.
        Permet de passer entre la vitesse effective v
        et les vitesses des roues u.
        """
        return 2 * self.M(theta)

    def velocity_to_wheel_speeds(self, v_new):
        """Vitesse effective v [m/s] -> vitesses de roues u = (droite, gauche) [m/s]."""
        # Si v sort du disque atteignable, on réduit sa norme sans changer sa direction :
        # saturer chaque roue séparément déformerait la trajectoire.
        speed = np.linalg.norm(v_new)
        if speed > V_MAX_EFF:
            v_new = v_new * V_MAX_EFF / speed

        return self.M_inv(self.theta) @ v_new

    # ------------------------------------------------------------------
    # Actionneur
    # ------------------------------------------------------------------
    def wheel_speeds_to_pwm(self, u):
        """Vitesses de roues [m/s] -> PWM entiers signés (droite, gauche)."""
        pwm = np.clip(K_PWM * u, -PWM_MAX, PWM_MAX)
        return int(pwm[0]), int(pwm[1])

    def send_pwm(self, pwm_d, pwm_g):
        """Formate et écrit la trame. Seule méthode qui connaît le protocole série."""
        cmd = f"0,S,{self.robot_id},M,{pwm_d},{pwm_g},\r\n"
        self.serial.write(cmd.encode("ascii"))

    def stop(self):
        self.send_pwm(0, 0)

    # ------------------------------------------------------------------
    # Orchestration
    # ------------------------------------------------------------------
    def send_velocity_command(self, v_new):
        """
        Envoi des données de vitesse effective au robot.
        Enchaîne : cinématique -> conversion PWM -> trame série.
        """
        u_new = self.velocity_to_wheel_speeds(v_new)
        pwm_d, pwm_g = self.wheel_speeds_to_pwm(u_new)
        self.send_pwm(pwm_d, pwm_g)

    def loop(self):
        """
        Main loop
        """
        # Le futur filtre ORCA s’insérera dans loop, entre compute_preferred_velocity et send_velocity_command, sans toucher au reste.
        # Sans pose récente, on ne sait plus où est le robot : on l'arrête.
        if not self.pose_is_fresh():
            self.get_logger().warning("Pose périmée : arrêt", throttle_duration_sec=1.0)
            self.stop()
            return

        self.update_waypoint()
        v_pref = self.compute_preferred_velocity()

        v_new = v_pref   # TODO : filtre ORCA ici (v_pref + autres robots -> v_new)
        self.send_velocity_command(v_new)


def main(args=None):
    # remove_ros_args : ignore les options "--ros-args ..." pour retrouver l'identifiant.
    argv = remove_ros_args(sys.argv)
    # 3 conditions d'erreur : pas d'argument, argument non numérique, argument hors plage.
    no_argument_passed = len(argv) < 2
    is_not_natural_numer = not argv[1].isdigit()
    id_is_out_of_range = not 1 <= int(argv[1]) <= 4
    
    if no_argument_passed or is_not_natural_numer or id_is_out_of_range:
        print("Usage : ros2 run <paquet> robot <id 1..4>")
        sys.exit(1)

    rclpy.init(args=args)
    node = RobotNode(int(argv[1]))

    # Boucle principale : rclpy.spin() fait tourner en boucle le traitement des callbacks 
    try: 
        rclpy.spin(node)
    # Stop si ctrl+C est préssé
    except KeyboardInterrupt:
        pass
    finally:
        # Moteurs coupés avant de libérer le port, sinon le robot garde sa consigne.
        node.stop()
        node.serial.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()