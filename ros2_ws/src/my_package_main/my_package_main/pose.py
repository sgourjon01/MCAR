"""
Mathias
Bouchut--Loubet
24/11/2026

Classe qui définit une pose d'un robot.
Elle permet de passer facilement entre les coordonnées
réelles et imaginaires.
"""

import numpy as np


class Pose:

    def __init__(self, robot_radius: float):
        # Rayon de robot
        self.r = robot_radius

        # Matrice de changement de représentation
        self.__A = 1 / (2 * self.r) * np.array([
            [self.r, self.r],
            [1, -1]
        ])
        self.__A_inv = np.array([
            [1, self.r],
            [1, -self.r]
        ])

        # Pose réel

        # Position
        self.x = 0
        self.y = 0
        self.theta = 0
        # Vitesse
        self.V = 0
        self.theta_dot = 0

        # Pose imaginaire

        # Position
        self.X = 0
        self.Y = 0
        # Vitesse
        self.X_dot = 0
        self.Y_dot = 0

    def __M(self):
        """
        Renvoi la matrice M.
        Permet de passer entre les vitesses des
        roues u et la vitesse holonome v.
        """
        c = np.cos(self.theta)
        s = np.sin(self.theta)

        M = 1/2 * np.array([
            [c-s, c+s],
            [c+s, s-c]
        ])

        return M

    def __M_inv(self):
        """
        Renvoi la matrice M inverse.
        Permet de passer entre la vitesse holonome v
        et les vitesses des roues u.
        """
        return 2 * self.__M()

    def __update_imag(self):
        """
        Met à jour la position imaginaire à partir
        des coordonnées réel.
        """
        self.X = self.x + self.r * np.cos(self.theta)
        self.Y = self.y + self.r * np.sin(self.theta)

        res = self.__M() @ self.__A_inv @ np.array([self.V, self.theta_dot])
        self.X_dot, self.Y_dot = res[0], res[1]

    def __update_real(self, theta):
        """
        Met à jour la position réel à partir
        des coordonnées imaginaires.
        """
        self.x = self.X - self.r * np.cos(self.theta)
        self.y = self.Y - self.r * np.sin(self.theta)
        self.theta = theta

        res = self.__A @ self.__M_inv() @ np.array([self.X_dot, self.Y_dot])
        self.V, self.theta_dot = res[0], res[1]

    def set_real(self, x: float, y: float, theta: float, V: float, theta_dot: float):
        """
        Assigne une nouvelle pose réelle.
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.V = V
        self.theta_dot = theta_dot

        self.__update_imag()

    def set_imag(self, X: float, Y: float, X_dot: float, Y_dot: float):
        """
        Assigne une nouvelle pose imaginaire.
        """
        self.X = X
        self.Y = Y
        self.X_dot = X_dot
        self.Y_dot = Y_dot

        self.__update_real(self.theta)

    def set(self, pose):
        """
        Assigne une nouvelle pose depuis une autre pose.
        """
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta
        self.V = pose.V
        self.theta_dot = pose.theta_dot
        self.X = pose.X
        self.Y = pose.Y
        self.X_dot = pose.X_dot
        self.Y_dot = pose.Y_dot


    # ----------------------------------------------------------------
    # Fonctions de calcul cinématqiue des vitesses des roues à partir
    # de la vitesse du robot virtuel (holonome)
    # ----------------------------------------------------------------
    def convert_virtual_speed_to_wheel_speeds(self, X_dot: float, Y_dot: float) -> tuple:
        """
        Calcule les vitesses des roues à partir de la vitesse
        holonome donnée au centre imaginaire du robot.

        Pre:
            Deux flottants pour la vitesse holonome (X_dot, Y_dot) dans
            le repère du sol en mm/s
        Post:
            Un couple de flottants (v_droite, v_gauche) en mm/s.
        """
        u = self.__M_inv() @ np.array([X_dot, Y_dot])

        return float(u[0]), float(u[1])