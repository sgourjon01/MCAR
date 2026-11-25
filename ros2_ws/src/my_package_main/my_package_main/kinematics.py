"""Cinématique d'un robot à entraînement différentiel (centre effectif)."""

import math

import numpy as np


class Kinematics:
    """
    Modèle cinématique d'un robot différentiel et de son centre effectif.
    On pilote un point virtuel, le « centre effectif », décalé de D devant
    le centre q dans la direction du cap. Ce point est holonome : 
    n'importe quelle vitesse peut lui être imposée, ce qui permet
    d'appliquer ORCA avec un disque de rayon R = r + D.

    Référence : Snape et al., « Smooth and Collision-Free Navigation for
    Multiple Robots Under Differential-Drive Constraints », IROS 2010.

    La classe est sans état et indépendante des unités : toutes les
    grandeurs d'entrée doivent simplement être exprimées dans le même
    système d'unités
    """

    def __init__(self, wheel_base: float, effective_offset: float,
                 robot_radius: float, max_wheel_speed: float):
        """
        Initialise le modèle.

        :param wheel_base: distance L entre les deux roues (> 0).
        :param effective_offset: décalage D du centre effectif (> 0).
        :param robot_radius: rayon r du robot (> 0).
        :param max_wheel_speed: vitesse maximale d'une roue, en valeur
            absolue (> 0).
        """
        self._wheel_base = float(wheel_base)
        self._offset = float(effective_offset)
        self._robot_radius = float(robot_radius)
        self._max_wheel_speed = float(max_wheel_speed)

    # ------------------------------------------------------------------
    # Propriétés (transforme une méthode en attrivbut calculé)
    # ------------------------------------------------------------------
    @property
    def effective_radius(self) -> float:
        """Rayon effectif R = r + D, à donner à ORCA."""
        return self._robot_radius + self._offset

    @property
    def max_effective_speed(self) -> float:
        """
        Vitesse max du centre effectif réalisable quel que soit le cap.

        C'est le rayon du cercle inscrit dans l'ensemble des vitesses
        atteignables. À utiliser comme vitesse maximale dans ORCA.
        """
        ratio = self._wheel_base / (2.0 * self._offset)
        return self._max_wheel_speed / math.sqrt(1.0 + ratio ** 2)

    # ------------------------------------------------------------------
    # Réel -> virtuel (holonome)
    # ------------------------------------------------------------------
    def effective_center(self, position: np.ndarray, theta: float) -> np.ndarray:
        """Retourne la position p du centre effectif : p = q + D*(cos, sin)."""
        return np.asarray(position, dtype=float) + self._offset * self._heading(theta)

    def effective_velocity(self, theta: float, linear_velocity: np.ndarray,
                           angular_velocity: float) -> np.ndarray:
        """
        Retourne la vitesse (X_dot, Y_dot) du centre effectif.

        X_dot = x_dot - D*sin(theta)*theta_dot
        Y_dot = y_dot + D*cos(theta)*theta_dot
        """
        return (np.asarray(linear_velocity, dtype=float)
                + self._offset * angular_velocity * self._normal(theta))

    # ------------------------------------------------------------------
    # Virtuel -> consigne roues
    # ------------------------------------------------------------------
    def wheel_speeds(self, theta: float, effective_velocity: np.ndarray):
        """
        Convertit une vitesse du centre effectif en vitesses de roues.

        Si la consigne dépasse la vitesse max d'une roue, les deux roues
        sont réduites dans la même proportion (la trajectoire est conservée,
        seule la vitesse diminue).

        :return: tuple (v_gauche, v_droite).
        """
        velocity = np.asarray(effective_velocity, dtype=float)

        linear = float(np.dot(velocity, self._heading(theta)))
        angular = float(np.dot(velocity, self._normal(theta))) / self._offset

        half_turn = 0.5 * self._wheel_base * angular
        v_left = linear - half_turn
        v_right = linear + half_turn

        peak = max(abs(v_left), abs(v_right))
        if peak > self._max_wheel_speed:
            scale = self._max_wheel_speed / peak
            v_left *= scale
            v_right *= scale

        return v_left, v_right

    # ------------------------------------------------------------------
    # Utilitaires
    # ------------------------------------------------------------------
    @staticmethod
    def _heading(theta: float) -> np.ndarray:
        """Vecteur unitaire dans la direction du cap.
            Utile pour projeter la vitesse linéaire sur l'axe du robot.
            #Simplifie les cosinus et sinus """
        return np.array([math.cos(theta), math.sin(theta)])

    @staticmethod
    def _normal(theta: float) -> np.ndarray:
        """Vecteur unitaire orthogonal au cap (à gauche).
            Utile pour projeter la vitesse linéaire sur l'axe du robot.
            #Simplifie les cosinus et sinus """

        return np.array([-math.sin(theta), math.cos(theta)])