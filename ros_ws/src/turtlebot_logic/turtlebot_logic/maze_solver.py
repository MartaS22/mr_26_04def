import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
 
# --- NUOVE LIBRERIE PER LA GEOMETRIA E LA MEMORIA SPAZIALE ---
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
 
class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.get_logger().info("🧠 Cervello Attivo: Memoria Spaziale TF2 inizializzata...")
        
        self.bridge = CvBridge()
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- INIZIALIZZAZIONE TF2 (Il GPS del robot) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.aruco_params = aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters()
            
        self.aruco_params.minMarkerPerimeterRate = 0.005
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
            
        # --- LA MEMORIA DEL LABIRINTO ---
        # Prima era un "set", ora è un "dictionary" per associare ID -> Coordinate
        self.mappa_chiavi = {} 
        self.totale_chiavi = 9
        
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
 
    def salva_coordinata_mappa(self, marker_id, tvec):
        """Traduce i metri visti dalla telecamera in coordinate (X,Y) sulla mappa globale"""
        try:
            # 1. Chiediamo a TF2 la posizione della telecamera rispetto alla mappa in questo esatto millisecondo
            # "map" è l'origine del labirinto, "camera_link" o "camera_rgb_optical_frame" è l'occhio del robot
            # Usa il frame ottico corretto del tuo URDF, di solito è 'camera_rgb_optical_frame'
            trans = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
            
            # 2. Creiamo un punto geometrico basato su ciò che vede la telecamera
            # Nelle telecamere ROS: X è destra/sinistra, Y è giù/su, Z è profondità (avanti)
            punto_camera = PointStamped()
            punto_camera.header.frame_id = 'camera_rgb_optical_frame'
            punto_camera.header.stamp = self.get_clock().now().to_msg()
            punto_camera.point.x = float(tvec[0][0])
            punto_camera.point.y = float(tvec[1][0])
            punto_camera.point.z = float(tvec[2][0]) # La distanza in metri
            
            # 3. La magia di ROS 2: applichiamo la trasformazione!
            punto_mappa = do_transform_point(punto_camera, trans)
            
            # 4. Salviamo la coordinata nella memoria!
            self.mappa_chiavi[marker_id] = (punto_mappa.point.x, punto_mappa.point.y)
            
            self.get_logger().info(
                f"🗺️ CHIAVE {marker_id} MAPPATA! Coordinate globali: X={punto_mappa.point.x:.2f}, Y={punto_mappa.point.y:.2f}"
            )
            
        except Exception as e:
            # Se la mappa non è ancora allineata (SLAM non pronto) ignoriamo temporaneamente
            self.get_logger().debug(f"Attesa allineamento mappa (TF2): {e}")
 
    def image_callback(self, msg):
        if self.camera_matrix is None: return
 
        try:
            cv_image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = np.array(cv_image_raw, copy=True, dtype=np.uint8)
            if cv_image is None or cv_image.size == 0: return
 
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            height, width = gray_image.shape
            zoomed_image = cv2.resize(gray_image, (width * 3, height * 3), interpolation=cv2.INTER_NEAREST)
            
            corners, ids, rejected = aruco.detectMarkers(zoomed_image, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                original_corners = [c / 3.0 for c in corners]
                
                for i, marker_id in enumerate(ids.flatten()):
                    if 1 <= marker_id <= self.totale_chiavi:
                        # Se la chiave NON è ancora nella nostra mappa mentale, calcoliamo la sua posizione
                        if marker_id not in self.mappa_chiavi:
                            success, rvec, tvec = cv2.solvePnP(
                                self.obj_points, original_corners[i][0], self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                            
                            if success:
                                # Chiamiamo la funzione che trasforma i metri in coordinate GPS!
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
 