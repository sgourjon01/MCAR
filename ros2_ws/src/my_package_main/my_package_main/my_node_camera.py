#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ==============================================================================
# CONFIGURATION DE LA CAMÉRA ET DES MARQUEURS
# ==============================================================================
CAMERA_INDEX = 0        # Index 0 (/dev/video0) validé via CAP_V4L2
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
TARGET_FPS = 30         # Cadence cible (en VM, 10-15 Hz est plus sûr si saturation CPU)
MAX_DT = 0.5            # Au-delà de 0.5 s d'absence, réinitialise l'historique
DETECT_SCALE = 0.5      # Facteur de réduction pour detectMarkers (1.0 = pleine résolution)
REF_ALPHA = 0.10        # Lissage du repère de référence (faible = très stable, ex. 0.05 - 0.2)
VELOCITY_ALPHA = 0.35   # Lissage des vitesses (0.0 = figé, 1.0 = brut sans filtre)
POS_ALPHA = 0.4         # Lissage des positions (0.0 = figé, 1.0 = brut sans filtre)
SHOW_WINDOW = True      # Mettre à False si cv2.imshow fait ramer ou geler la VM
WINDOW_NAME = "Camera - Suivi ArUco"
WHEEL_OFFSET_X = 0.00   # Offset en millimètres dans le repère natif de l'ArUco robot
WHEEL_OFFSET_Y = 0.00   # Offset en millimètres dans le repère natif de l'ArUco robot

CAMERA_MATRIX = np.array([
    [2.28765453e+03, 0.00000000, 1.90488413e+03],
    [0.00000000, 2.29132838e+03, 1.11413392e+03],
    [0.00000000, 0.00000000, 1.00000000]
], dtype=np.float32)

DIST_COEFFS = np.array(
    [[0.17399552, -0.39706351, 0.00062571, -0.00082353, 0.19697345]],
    dtype=np.float32
)

MARKER_SIZE = 93.0      # Taille réelle des marqueurs en millimètres (9.3 cm)
REFERENCE_ID = 0        # ID de l'ArUco fixe servant d'origine et de plan au plateau

# Association explicite : {id_aruco: numero_robot}
ROBOT_MAPPING = {
    1: 1,
    2: 2,
    3: 3,
    4: 4,
}


def smooth_rotation_matrix(R_prev: np.ndarray, R_curr: np.ndarray, alpha: float) -> np.ndarray:
    """Mélange deux matrices de rotation et projette le résultat sur SO(3) par SVD."""
    R_blend = (1.0 - alpha) * R_prev + alpha * R_curr
    U, _, Vt = np.linalg.svd(R_blend)
    R_smooth = U @ Vt
    if np.linalg.det(R_smooth) < 0:
        U[:, -1] *= -1.0
        R_smooth = U @ Vt
    return R_smooth


class CameraNode(Node):
    def __init__(self):
        super().__init__('my_node_camera')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_pose', 10)
        self.cap = None
        self.timer = None
        self.history = {}
        self.ref_state = None   # {'R': np.ndarray, 't': np.ndarray, 'time': float}
        self._processing = False
        self._window_created = False

        # 1. Initialisation de la caméra via V4L2 avec flux compressé MJPG
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
        self.get_logger().info(
            f"Caméra ouverte (index {CAMERA_INDEX}, résolution {actual_w}x{actual_h})."
        )

        # 2. Configuration du détecteur ArUco (compatible OpenCV < 4.7 et >= 4.7)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, cv2.aruco.DetectorParameters()
            )
            self._use_new_aruco_api = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self._use_new_aruco_api = False

        # Critères d'arrêt pour l'affinage sub-pixel des coins
        self.subpix_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
        )

        # Modèle 3D d'un marqueur carré dans son propre repère local (Z = 0)
        s = MARKER_SIZE / 2.0
        self.object_points = np.array([
            [-s,  s, 0.0],  # Coin haut-gauche
            [ s,  s, 0.0],  # Coin haut-droit
            [ s, -s, 0.0],  # Coin bas-droit
            [-s, -s, 0.0]   # Coin bas-gauche
        ], dtype=np.float32)

        # 3. Démarrage de la boucle périodique ROS 2
        self.timer_period = 1.0 / TARGET_FPS
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(f"Nœud caméra démarré à {TARGET_FPS:.1f} Hz.")

    def _show_frame(self, frame: np.ndarray):
        """Affiche le retour vidéo sans faire planter le nœud en cas d'erreur X11/Wayland."""
        if not SHOW_WINDOW:
            return
        try:
            cv2.imshow(WINDOW_NAME, frame)
            cv2.waitKey(1)
            self._window_created = True
        except cv2.error as exc:
            self.get_logger().warn(f"Affichage OpenCV désactivé suite à une erreur : {exc}")
            self._window_created = False

    def _detect_aruco(self, gray: np.ndarray):
        """Détecte les marqueurs (éventuellement sur image réduite) puis affine les coins en pleine résolution."""
        if 0.1 <= DETECT_SCALE < 0.999:
            small = cv2.resize(
                gray, (0, 0), fx=DETECT_SCALE, fy=DETECT_SCALE, interpolation=cv2.INTER_LINEAR
            )
            if self._use_new_aruco_api:
                corners, ids, _ = self.detector.detectMarkers(small)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    small, self.dictionary, parameters=self.aruco_params
                )
            if corners:
                inv_scale = 1.0 / DETECT_SCALE
                corners = [c.astype(np.float32) * inv_scale for c in corners]
        else:
            if self._use_new_aruco_api:
                corners, ids, _ = self.detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, self.dictionary, parameters=self.aruco_params
                )

        # Affinage sub-pixel sur l'image niveaux de gris pleine résolution pour stabiliser solvePnP
        if corners:
            refined_corners = []
            for c in corners:
                c_sub = cv2.cornerSubPix(
                    gray,
                    c.reshape(-1, 1, 2).astype(np.float32),
                    winSize=(5, 5),
                    zeroZone=(-1, -1),
                    criteria=self.subpix_criteria
                )
                refined_corners.append(c_sub.reshape(1, 4, 2))
            corners = refined_corners

        return corners, ids

    def _project_to_pixel(self, pt_3d_cam: np.ndarray):
        """Projette un point 3D (repère caméra) vers des coordonnées pixels (u, v) entières."""
        pts_2d, _ = cv2.projectPoints(
            pt_3d_cam.reshape(1, 1, 3).astype(np.float32),
            np.zeros((3, 1), dtype=np.float32),
            np.zeros((3, 1), dtype=np.float32),
            CAMERA_MATRIX,
            DIST_COEFFS
        )
        u, v = pts_2d.reshape(2)
        return int(round(u)), int(round(v))

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
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = self._detect_aruco(gray)

            if ids is None:
                self._show_frame(frame)
                return

            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids_flat)

            # Estimation de la pose 3D (rvec, tvec) de chaque marqueur dans le repère caméra
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
                    detected_poses[int(marker_id)] = {
                        'rvec': rvec,
                        'tvec': tvec.reshape(3, 1),
                        'corner': corner
                    }

            # 1. Vérification de la présence du marqueur de référence du plateau
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

            # 2. Lissage et construction du changement de repère 3D : Caméra -> Plan de référence
            ref_data = detected_poses[REFERENCE_ID]
            R_ref_raw, _ = cv2.Rodrigues(ref_data['rvec'])
            t_ref_raw = ref_data['tvec']

            if (
                self.ref_state is None
                or (current_time - self.ref_state['time']) > MAX_DT
            ):
                R_ref = R_ref_raw
                t_ref = t_ref_raw
            else:
                R_ref = smooth_rotation_matrix(self.ref_state['R'], R_ref_raw, REF_ALPHA)
                t_ref = (1.0 - REF_ALPHA) * self.ref_state['t'] + REF_ALPHA * t_ref_raw

            self.ref_state = {
                'R': R_ref,
                't': t_ref,
                'time': current_time
            }

            R_ref_T = R_ref.T       # Transpose = inverse pour une matrice de rotation orthogonale
            rvec_ref_smooth, _ = cv2.Rodrigues(R_ref)

            # Dessine les axes du repère plateau lissé sur ID 0 (+X en rouge, +Y inversé en vert)
            px_ref_o = self._project_to_pixel(t_ref)
            px_ref_x = self._project_to_pixel(t_ref + R_ref @ np.array([[MARKER_SIZE * 0.75], [0.0], [0.0]], dtype=np.float32))
            px_ref_y = self._project_to_pixel(t_ref + R_ref @ np.array([[0.0], [-MARKER_SIZE * 0.75], [0.0]], dtype=np.float32))
            cv2.arrowedLine(frame, px_ref_o, px_ref_x, (0, 0, 255), 2, tipLength=0.2)
            cv2.arrowedLine(frame, px_ref_o, px_ref_y, (0, 255, 0), 2, tipLength=0.2)
            _ = rvec_ref_smooth

            # 3. Projection et publication individuelle de chaque robot détecté
            for marker_id, data in detected_poses.items():
                if marker_id not in ROBOT_MAPPING:
                    continue

                robot_num = ROBOT_MAPPING[marker_id]

                cv2.drawFrameAxes(
                    frame, CAMERA_MATRIX, DIST_COEFFS, data['rvec'], data['tvec'], MARKER_SIZE * 0.5
                )

                # Position 3D exprimée dans le repère natif de l'ArUco de référence (en mm)
                pos_rel = R_ref_T @ (data['tvec'] - t_ref)
                x_aruco = float(pos_rel[0, 0])
                y_aruco = float(pos_rel[1, 0])

                # Orientation relative native du robot dans le repère de l'ArUco de référence
                R_rob, _ = cv2.Rodrigues(data['rvec'])
                R_rel = R_ref_T @ R_rob
                theta_aruco = float(np.arctan2(R_rel[1, 0], R_rel[0, 0]))

                # Application de l'offset dans le repère natif de l'ArUco du robot
                cos_th = np.cos(theta_aruco)
                sin_th = np.sin(theta_aruco)
                raw_x_aruco = float(x_aruco + WHEEL_OFFSET_X * cos_th - WHEEL_OFFSET_Y * sin_th)
                raw_y_aruco = float(y_aruco + WHEEL_OFFSET_X * sin_th + WHEEL_OFFSET_Y * cos_th)

                # Passage au nouveau repère plateau avec l'axe Y inversé (Y_plateau = -Y_aruco)
                raw_x = raw_x_aruco
                raw_y = -raw_y_aruco
                theta_rel = -theta_aruco

                # Filtrage passe-bas sur (x, y) puis calcul des vitesses filtrées
                x_rel, y_rel = raw_x, raw_y
                v_x, v_y, omega = 0.0, 0.0, 0.0

                if robot_num in self.history:
                    prev = self.history[robot_num]
                    dt = current_time - prev['time']
                    if 0.0 < dt <= MAX_DT:
                        x_rel = float(POS_ALPHA * raw_x + (1.0 - POS_ALPHA) * prev['x'])
                        y_rel = float(POS_ALPHA * raw_y + (1.0 - POS_ALPHA) * prev['y'])

                        raw_vx = (x_rel - prev['x']) / dt
                        raw_vy = (y_rel - prev['y']) / dt
                        d_theta = np.arctan2(
                            np.sin(theta_rel - prev['theta']),
                            np.cos(theta_rel - prev['theta'])
                        )
                        raw_omega = d_theta / dt

                        v_x = float(VELOCITY_ALPHA * raw_vx + (1.0 - VELOCITY_ALPHA) * prev['vx'])
                        v_y = float(VELOCITY_ALPHA * raw_vy + (1.0 - VELOCITY_ALPHA) * prev['vy'])
                        omega = float(VELOCITY_ALPHA * raw_omega + (1.0 - VELOCITY_ALPHA) * prev['omega'])

                self.history[robot_num] = {
                    'x': x_rel,
                    'y': y_rel,
                    'theta': theta_rel,
                    'vx': v_x,
                    'vy': v_y,
                    'omega': omega,
                    'time': current_time
                }

                # Publication individuelle : [robot_id, x, y, theta, v_x, v_y, omega]
                msg = Float32MultiArray()
                msg.data = [
                    float(robot_num),
                    x_rel,
                    y_rel,
                    theta_rel,
                    v_x,
                    v_y,
                    omega
                ]
                self.publisher_.publish(msg)

                # Point 3D de l'entre-axe dans le repère de l'ArUco robot, converti en repère caméra
                offset_local_3d = np.array([[WHEEL_OFFSET_X], [WHEEL_OFFSET_Y], [0.0]], dtype=np.float32)
                t_wheel_cam = data['tvec'] + R_rob @ offset_local_3d

                # Point devant l'entre-axe (50 mm selon X local) pour tracer la flèche d'orientation
                heading_local_3d = np.array([[WHEEL_OFFSET_X + 50.0], [WHEEL_OFFSET_Y], [0.0]], dtype=np.float32)
                t_heading_cam = data['tvec'] + R_rob @ heading_local_3d

                px_aruco = self._project_to_pixel(data['tvec'])
                px_wheel = self._project_to_pixel(t_wheel_cam)
                px_heading = self._project_to_pixel(t_heading_cam)

                # Trait reliant le centre de l'ArUco à l'entre-axe des roues
                cv2.line(frame, px_aruco, px_wheel, (255, 200, 0), 2)
                # Point bleu = centre de l'ArUco
                cv2.circle(frame, px_aruco, 5, (255, 0, 0), -1)
                # Cercle magenta + flèche cyan = position réelle et cap du robot (entre-axe)
                cv2.circle(frame, px_wheel, 7, (255, 0, 255), -1)
                cv2.circle(frame, px_wheel, 9, (255, 255, 255), 1)
                cv2.arrowedLine(frame, px_wheel, px_heading, (255, 255, 0), 2, tipLength=0.35)

                cv2.putText(
                    frame,
                    f"R{robot_num}: x={x_rel:.1f}mm y={y_rel:.1f}mm th={theta_rel:.2f}rad",
                    (px_wheel[0] + 12, px_wheel[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

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