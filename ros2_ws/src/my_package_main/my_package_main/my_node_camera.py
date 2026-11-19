#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Configuration de la caméra et des marqueurs
CAMERA_INDEX = 0        # Index 0 (/dev/video0) validé via CAP_V4L2
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
TARGET_FPS = 30       # Cadence prudente en VM (10-15 Hz) pour éviter la saturation
MAX_DT = 0.5            # Au-delà de 0.5 s d'absence, on réinitialise l'historique vitesse
DETECT_SCALE = 0.5      # Facteur de réduction pour detectMarkers (1.0 = pleine résolution)
SHOW_WINDOW = True      # Mettre à False si cv2.imshow fait ramer ou geler la VM
WINDOW_NAME = "Camera - Suivi ArUco"

CAMERA_MATRIX = np.array([
    [2.28765453e+03, 0.00000000, 1.90488413e+03],
    [0.00000000, 2.29132838e+03, 1.11413392e+03],
    [0.00000000, 0.00000000, 1.00000000]
], dtype=np.float32)

DIST_COEFFS = np.array(
    [[0.17399552, -0.39706351, 0.00062571, -0.00082353, 0.19697345]],
    dtype=np.float32
)

MARKER_SIZE = 0.102     # Taille réelle en mètres (10.2 cm)
REFERENCE_ID = 0        # ID de l'ArUco dans le coin servant de repère plateau

# Association explicite : {id_aruco: numero_robot}
ROBOT_MAPPING = {
    1: 1,
    2: 2,
    3: 3,
    4: 4,
}


def rotation_to_euler(rvec):
    """Convertit un vecteur de rotation rvec en angles d'Euler (roll, pitch, yaw) en radians."""
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


class CameraNode(Node):
    def __init__(self):
        super().__init__('my_node_camera')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_data', 10)
        self.cap = None
        self.timer = None
        self.history = {}
        self._processing = False
        self._window_created = False

        # Initialisation de la caméra via V4L2 avec résolution et flux compressé MJPG
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"Impossible d'ouvrir la caméra à l'index {CAMERA_INDEX} !")
            cap.release()
            return

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.cap = cap

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Caméra ouverte (index {CAMERA_INDEX}, résolution {actual_w}x{actual_h}).")

        # Configuration ArUco (DICT_5X5_50) compatible anciennes et nouvelles versions d'OpenCV
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
            self._use_new_aruco_api = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self._use_new_aruco_api = False

        s = MARKER_SIZE / 2.0
        self.object_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float32)

        self.timer_period = 1.0 / TARGET_FPS
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(f"Nœud caméra démarré à {TARGET_FPS:.1f} Hz.")

    def _show_frame(self, frame):
        if not SHOW_WINDOW:
            return
        try:
            cv2.imshow(WINDOW_NAME, frame)
            cv2.waitKey(1)
            self._window_created = True
        except cv2.error as exc:
            self.get_logger().warn(f"Affichage OpenCV désactivé suite à une erreur : {exc}")
            self._window_created = False

    def _detect_aruco(self, frame):
        if 0.1 <= DETECT_SCALE < 0.999:
            small = cv2.resize(frame, (0, 0), fx=DETECT_SCALE, fy=DETECT_SCALE, interpolation=cv2.INTER_LINEAR)
            if self._use_new_aruco_api:
                corners, ids, _ = self.detector.detectMarkers(small)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(small, self.dictionary, parameters=self.aruco_params)
            if corners:
                inv_scale = 1.0 / DETECT_SCALE
                corners = [c.astype(np.float32) * inv_scale for c in corners]
            return corners, ids

        if self._use_new_aruco_api:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.aruco_params)
        return corners, ids

    def timer_callback(self):
        if self._processing or self.cap is None or not self.cap.isOpened():
            return

        self._processing = True
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn("Impossible de lire l'image de la caméra")
                return

            current_time = time.monotonic()
            corners, ids = self._detect_aruco(frame)

            if ids is None:
                self._show_frame(frame)
                return

            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids_flat)

            detected_poses = {}

            for corner, marker_id in zip(corners, ids_flat):
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    corner.reshape(4, 2),
                    CAMERA_MATRIX,
                    DIST_COEFFS,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    x_cam, y_cam, z_cam = tvec.flatten()
                    _, _, yaw = rotation_to_euler(rvec)
                    detected_poses[int(marker_id)] = {
                        'x': float(x_cam),
                        'y': float(y_cam),
                        'z': float(z_cam),
                        'yaw': float(yaw),
                        'corner': corner
                    }

            # 1. Vérification du marqueur de référence du plateau
            if REFERENCE_ID not in detected_poses:
                cv2.putText(
                    frame,
                    f"ATTENTION: Repere plateau (ID {REFERENCE_ID}) introuvable !",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2
                )
                self._show_frame(frame)
                return

            ref_data = detected_poses[REFERENCE_ID]
            ref_x, ref_y = ref_data['x'], ref_data['y']
            ref_yaw = ref_data['yaw']

            cos_r = np.cos(ref_yaw)
            sin_r = np.sin(ref_yaw)

            # Tableau à plat : [robot_id, x, y, theta, v_x, v_y, omega, ...]
            msg_data = []

            # 2. Traitement uniquement des marqueurs associés aux robots
            for marker_id, data in detected_poses.items():
                if marker_id not in ROBOT_MAPPING:
                    continue

                robot_num = ROBOT_MAPPING[marker_id]

                dx = data['x'] - ref_x
                dy = data['y'] - ref_y

                x_rel = float(dx * cos_r + dy * sin_r)
                y_rel = float(-dx * sin_r + dy * cos_r)
                theta_rel = float(np.arctan2(
                    np.sin(data['yaw'] - ref_yaw),
                    np.cos(data['yaw'] - ref_yaw)
                ))

                v_x, v_y, omega = 0.0, 0.0, 0.0
                if robot_num in self.history:
                    prev = self.history[robot_num]
                    dt = current_time - prev['time']
                    if 0.0 < dt <= MAX_DT:
                        v_x = float((x_rel - prev['x']) / dt)
                        v_y = float((y_rel - prev['y']) / dt)
                        d_theta = np.arctan2(
                            np.sin(theta_rel - prev['theta']),
                            np.cos(theta_rel - prev['theta'])
                        )
                        omega = float(d_theta / dt)

                self.history[robot_num] = {
                    'x': x_rel,
                    'y': y_rel,
                    'theta': theta_rel,
                    'time': current_time
                }

                msg_data.extend([
                    float(robot_num),
                    x_rel,
                    y_rel,
                    theta_rel,
                    v_x,
                    v_y,
                    omega
                ])

                cx, cy = data['corner'].reshape(4, 2).mean(axis=0).astype(int)
                cv2.putText(
                    frame,
                    f"Robot {robot_num} (ID {marker_id}): x={x_rel:.2f} y={y_rel:.2f}",
                    (cx, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2
                )

            # 3. Publication sur le topic /robot_data
            if len(msg_data) > 0:
                msg = Float32MultiArray()
                msg.data = msg_data
                self.publisher_.publish(msg)

            self._show_frame(frame)
        finally:
            self._processing = False

    def destroy_node(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        if self._window_created:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()