"""
Thomas
Eyer
28/11/2026

Classe qui regroupe tout ce qui touche au matériel d'un robot :
moteurs des roues et leds.
"""

import math
import os
import serial 

# Constantes de l'actionneur
K_PWM = 1 # PWM par mm/s de vitesse de roue 
PWM_MAX = 100 # PWM maximum en valeur absolue
SERIAL_BAUDRATE = 115200 # Vitesse du port série (bauds)
PORT_COM = os.environ['SMARTIES_PORT_COM'] # Port série du robot (ex: /dev/ttyUSB0)



class RobotDriver:
    """
    Classe qui pilote le matériel d'un robot.
    """

    def __init__(self, robot_id: int, logger=None):
        self.robot_id = robot_id
        self.logger = logger
        # ouverture du port série pour communiquer avec le robot
        self.serial = serial.Serial(PORT_COM, SERIAL_BAUDRATE, timeout=0.1)


    def send_wheel_speeds(self, v_right: float, v_left: float):
        """
        Envoie la consigne de vitesse des roues au robot.
        Pre:
            Deux flottants, les vitesses des roues droite et gauche en
            mm/s (signées, positives vers l'avant).
        """
        # Sécurité : une valeur non finie arrête le robot
        if not (math.isfinite(v_right) and math.isfinite(v_left)):
            self.__warn("Consigne roues non finie, arrêt du robot.")
            v_right = v_left = 0.0

        # mm/s -> PWM entiers signés, saturés à PWM_MAX
        pwm_right, pwm_left = (
            int(round(max(-PWM_MAX, min(PWM_MAX, K_PWM * v))))
            for v in (v_right, v_left)
        )

        # Trame série
        frame = f"0,S,{self.robot_id},M,{pwm_right},{pwm_left},\r\n"

        try:
            self.serial.write(frame.encode("ascii"))
        except serial.SerialException as error:
            self.__warn(f"Échec d'écriture série : {error}")

        self.__debug(f"Consigne roues : droite={v_right:.1f} gauche={v_left:.1f} mm/s")


    def stop(self):
        """
        Arrête les roues du robot.
        """
        self.send_wheel_speeds(0.0, 0.0)


    def set_leds(self, color: tuple):
        """
        Allume les leds du robot d'une couleur donnée.
        Pre:
            Un triplet d'entiers (R, V, B) compris entre 0 et 255.
            (0, 0, 0) éteint les leds.
        """
        if len(color) != 3 or any(not 0 <= c <= 255 for c in color):
            raise ValueError(f"Couleur invalide : {color}")

        # TODO, envoyer la couleur aux leds
        self.__debug(f"Leds : {tuple(int(c) for c in color)}")

    def close(self):
        """
        Arrête le robot et libère le port série.
        """
        self.stop()
        self.set_leds(0, 0, 0)
        if self.serial is not None:
            self.serial.close()

    def __debug(self, text: str):
        if self.logger is not None:
            self.logger.debug(text)

    def __warn(self, text: str):
        if self.logger is not None:
            self.logger.warning(text)