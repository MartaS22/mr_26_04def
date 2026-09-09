import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
 
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
 
# --- NUOVE LIBRERIE PER LA NAVIGAZIONE AUTONOMA ---
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time
 
class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.get_logger().info("🧠 Cervello Attivo: Inizializzazione Navigazione e Memoria...")
        
        self.bridge = CvBridge()
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.aruco_params = aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters()
            
        self.aruco_params.minMarkerPerimeterRate = 0.005
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
            
        self.mappa_chiavi = {} 
        self.totale_chiavi = 9
        
        # --- STATI DELLA MACCHINA ---
        self.current_state = 'ESPLORAZIONE'
        self.frame_count = 0
        
        # --- CONFIGURAZIONE NAV2 ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Punti di ronda per forzare il robot a girare tutto il labirinto
        self.waypoints_esplorazione = [
            (2.0, 2.0),   # Angolo Alto-Destra
            (-2.0, 2.0),  # Angolo Alto-Sinistra
            (-2.0, -2.0), # Angolo Basso-Sinistra
            (2.0, -2.0),  # Angolo Basso-Destra
            (0.0, 0.0)    # Ritorno al Centro
        ]
        self.wp_index = 0
        
        self.marker_size = 0.30
        half = self.marker_size / 2.0
        self.obj_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
 
        # Aspettiamo 3 secondi per far caricare bene Gazebo e Nav2, poi avviamo l'esplorazione
        self.timer = self.create_timer(3.0, self.avvia_esplorazione)
 
    def avvia_esplorazione(self):
        self.timer.cancel() # Eseguiamo questa funzione una volta sola
        self.get_logger().info("🚀 AVVIO ESPLORAZIONE AUTONOMA DEL LABIRINTO!")
        self.vai_al_prossimo_waypoint()
 
    def vai_al_prossimo_waypoint(self):
        if self.current_state != 'ESPLORAZIONE':
            return
 
        if self.wp_index < len(self.waypoints_esplorazione):
            x, y = self.waypoints_esplorazione[self.wp_index]
            self.get_logger().info(f"🧭 In viaggio verso l'area: X={x}, Y={y}")
            self.invia_goal_nav2(x, y)
            self.wp_index += 1
        else:
            self.get_logger().info("🔄 Finito il giro di ronda, ricomincio per cercare chiavi mancanti...")
            self.wp_index = 0
            self.vai_al_prossimo_waypoint()
 
    def invia_goal_nav2(self, x, y):
        # Aspettiamo che il server di navigazione sia pronto
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0 # Rotazione base
 
        # Inviamo il comando e diciamo a Python quali funzioni chiamare quando finisce
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Nav2 ha rifiutato la destinazione (forse è dentro un muro). Salto al prossimo!")
            self.vai_al_prossimo_waypoint()
            return
 
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
 
    def get_result_callback(self, future):
        # Quando arriva a destinazione, chiamiamo il prossimo waypoint
        if self.current_state == 'ESPLORAZIONE':
            self.get_logger().info("✅ Area raggiunta. Procedo alla successiva.")
            self.vai_al_prossimo_waypoint()
 
    # ==========================================
    # LOGICA DI VISIONE (Rimane quasi identica)
    # ==========================================
    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
 
    def salva_coordinata_mappa(self, marker_id, tvec):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
            
            punto_camera = PointStamped()
            punto_camera.header.frame_id = 'camera_rgb_optical_frame'
            punto_camera.header.stamp = self.get_clock().now().to_msg()
            punto_camera.point.x, punto_camera.point.y, punto_camera.point.z = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
            
            punto_mappa = do_transform_point(punto_camera, trans)
            self.mappa_chiavi[marker_id] = (punto_mappa.point.x, punto_mappa.point.y)
            
            self.get_logger().info(f"🗺️ CHIAVE {marker_id} REGISTRATA! Coordinate: X={punto_mappa.point.x:.2f}, Y={punto_mappa.point.y:.2f} (Totale: {len(self.mappa_chiavi)}/{self.totale_chiavi})")
            
            # --- CONDIZIONE DI VITTORIA DELLA FASE 1 ---
            if len(self.mappa_chiavi) == self.totale_chiavi and self.current_state == 'ESPLORAZIONE':
                self.get_logger().info("🏆 TUTTE LE 9 CHIAVI SONO STATE MAPPATE! Fermo l'esplorazione casuale.")
                self.current_state = 'RACCOLTA_SEQUENZIALE'
                # Qui bloccheremo il robot per iniziare il giro di raccolta 1->9!
                
        except Exception as e:
            pass
 
    def image_callback(self, msg):
        if self.camera_matrix is None: return
        self.frame_count += 1
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
                        if marker_id not in self.mappa_chiavi:
                            success, rvec, tvec = cv2.solvePnP(self.obj_points, original_corners[i][0], self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                            if success:
                                self.salva_coordinata_mappa(marker_id, tvec)
        except Exception as e:
            pass
 
def main(args=None):
    rclpy.init(args=args)
    node = MazeSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 