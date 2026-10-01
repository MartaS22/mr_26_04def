import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from enum import Enum
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class RobotState(Enum):

    EXPLORING = 1
    APPROACHING_KEY = 2

class MazeSolverNode(Node):

    def __init__(self):

        super().__init__('maze_solver_node')
        self.get_logger().info(" Cervello Attivo: Logica di Raccolta e Memoria Spaziale inizializzate...")
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.explore_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.log_pub = self.create_publisher(String, '/maze_keys_log', 10)

        try:

            self.aruco_params = aruco.DetectorParameters_create()

        except AttributeError:

            self.aruco_params = aruco.DetectorParameters()

        self.aruco_params.minMarkerPerimeterRate = 0.005
        self.totale_chiavi = 9
        self.chiave_target = 1
        self.stato_corrente = RobotState.EXPLORING
        self.mappa_chiavi = {}
        self.marker_size = 0.30
        half = self.marker_size / 2.0

        self.obj_points = np.array([

            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]  

        ], dtype=np.float32)

    def info_callback(self, msg):

        if self.camera_matrix is None:

            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def set_explore_state(self, resume: bool):

        msg = Bool()
        msg.data = resume
        self.explore_pub.publish(msg)
        stato_str = "RIPRESA" if resume else "IN PAUSA"
        self.get_logger().info(f" Esplorazione autonoma: {stato_str}")

    def controlla_memoria(self):

        """Controlla se la prossima chiave da cercare è già stata vista in passato."""

        if self.chiave_target in self.mappa_chiavi:

            self.get_logger().info(f" MI RICORDO DOVE SI TROVA LA CHIAVE {self.chiave_target}! Vado dritto all'obiettivo.")

            # Recupero le coordinate salvate
            map_x, map_y = self.mappa_chiavi[self.chiave_target]

            # Imposto lo stato e mi dirigo verso le coordinate
            self.set_explore_state(False)
            self.stato_corrente = RobotState.APPROACHING_KEY
            self.go_to_key(map_x, map_y)

            return True

        return False

    def raccogli_chiave(self):

        """Funzione chiamata quando il robot è fisicamente vicino alla chiave"""

        if self.stato_corrente != RobotState.APPROACHING_KEY:

            return

        self.get_logger().info(f" CHIAVE {self.chiave_target} RACCOLTA!")

        # 0. Data logging
        log_msg = String()
        log_msg.data = f"Chiave {self.chiave_target} raccolta!"
        self.log_pub.publish(log_msg)


        # 1. Fermiamo Nav2

        if self.goal_handle is not None:

            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        # 2. Passiamo alla chiave successiva

        self.chiave_target += 1

        # 3. Controllo fine missione

        if self.chiave_target > self.totale_chiavi:

            self.get_logger().info(" OBBIETTIVO RAGGIUNTO! TUTTE LE 9 CHIAVI SONO STATE RACCOLTE!")

            return

        # 4. Consultiamo la memoria prima di tornare a esplorare casualmente!

        if not self.controlla_memoria():

            self.get_logger().info(f" Memoria vuota per la chiave {self.chiave_target}. Avvio esplorazione...")
            self.stato_corrente = RobotState.EXPLORING
            self.set_explore_state(True)

    def go_to_key(self, x, y):

        if not self.nav_client.wait_for_server(timeout_sec=2.0):

            self.get_logger().error("Nav2 non è pronto!")

            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 

        self.get_logger().info(f" In navigazione verso la chiave {self.chiave_target} alle coordinate X:{x:.2f} Y:{y:.2f}...")
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:

            self.get_logger().warn("Goal rifiutato da Nav2! Riprendo a esplorare...")
            self.stato_corrente = RobotState.EXPLORING
            self.set_explore_state(True)

            return

        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        """Si attiva se Nav2 termina da solo (perché è arrivato o perché ha abortito sbattendo)"""

        if self.stato_corrente == RobotState.APPROACHING_KEY:

            status = future.result().status
            self.get_logger().info(f"Nav2 ha terminato l'avvicinamento (Status: {status}).")
            self.raccogli_chiave()

    def image_callback(self, msg):

        if self.camera_matrix is None: return

        try:

            cv_image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image_raw, cv2.COLOR_BGR2GRAY)
            height, width = gray_image.shape
            zoomed_image = cv2.resize(gray_image, (width * 3, height * 3), interpolation=cv2.INTER_NEAREST)
            corners, ids, rejected = aruco.detectMarkers(zoomed_image, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:

                original_corners = [c / 3.0 for c in corners]

                for i, marker_id in enumerate(ids.flatten()):


                    if 1 <= marker_id <= self.totale_chiavi:

                        success, rvec, tvec = cv2.solvePnP(

                            self.obj_points, original_corners[i][0], self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                        if success:

                            # Troviamo la distanza

                            distanza_metri = float(tvec[2][0])

                            # Calcoliamo SEMPRE le coordinate sulla mappa, ci servono sia per andare che per ricordare

                            try:

                                trans = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
                                punto_camera = PointStamped()
                                punto_camera.header.frame_id = 'camera_rgb_optical_frame'
                                punto_camera.point.x = float(tvec[0][0])
                                punto_camera.point.y = float(tvec[1][0])
                                punto_camera.point.z = float(tvec[2][0])
                                punto_mappa = do_transform_point(punto_camera, trans)
                                map_x, map_y = punto_mappa.point.x, punto_mappa.point.y

                                # CASO 1: identificazione chiave che stiamo cercando!

                                if marker_id == self.chiave_target:

                                    if self.stato_corrente == RobotState.EXPLORING:
                                        self.set_explore_state(False)
                                        self.stato_corrente = RobotState.APPROACHING_KEY
                                        self.go_to_key(map_x, map_y)
                                        break

                                    elif self.stato_corrente == RobotState.APPROACHING_KEY:

                                        if distanza_metri <= 0.65:
                                            self.raccogli_chiave()
                                            break

                                # CASO 2: Non è la chiave che cerco, la salvo in memoria!

                                else:

                                    # Se non l'ho ancora salvata, la memorizzo per il futuro
                                    if marker_id > self.chiave_target and marker_id not in self.mappa_chiavi:

                                        self.mappa_chiavi[marker_id] = (map_x, map_y)
                                        self.get_logger().info(f" CHIAVE {marker_id} avvistata! Memorizzo coordinate (X:{map_x:.2f}, Y:{map_y:.2f}) per il futuro.")

                            except Exception as e:

                                # Se le TF non sono pronte, ignora questo frame

                                pass

        except Exception as e:

            pass

def main(args=None):

    rclpy.init(args=args)
    node = MazeSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':

    main()
 