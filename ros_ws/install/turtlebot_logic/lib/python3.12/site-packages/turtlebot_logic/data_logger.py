import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import csv
import os
import math
from datetime import datetime

class DataLoggerNode(Node):
    
    # 1. CORRETTO: Aggiunti i doppi trattini bassi __init__
    def __init__(self):
        super().__init__('data_logger_node')
        self.get_logger().info("Avvio Data Logger: Registrazione missione in corso...")

        # Variabili di stato correnti
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_v_lin = 0.0
        self.current_v_ang = 0.0
        self.min_lidar_distance = 0.0
        self.last_camera_timestamp = "N/A"
        self.keys_logged = ""
        
        # Sottoscrizioni ai sensori e al movimento
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Sottoscrizione per ricevere i dati delle chiavi dal maze_solver
        self.create_subscription(String, '/maze_keys_log', self.keys_callback, 10)

        # Preparazione del file CSV di output
        log_dir = "/ros_ws/log_missione"
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime("robot_log_%Y%m%d_%H%M%S.csv")
        self.csv_file_path = os.path.join(log_dir, filename)
        
        self.file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.file)
        
        # Intestazione del file (Colonne)
        self.csv_writer.writerow([
            'Timestamp_Sistema', 
            'Posizione_X', 
            'Posizione_Y', 
            'Velocita_Lineare', 
            'Velocita_Angolare', 
            'Ostacolo_Piu_Vicino_Lidar(m)', 
            'Stato_Telecamera',
            'Eventi_Chiavi'
        ])

        # Timer: Scrive i dati sul file 2 volte al secondo (2.0 Hz)
        self.timer = self.create_timer(0.5, self.save_data_to_csv)

    def odom_callback(self, msg):
        # Estrae la posizione e il percorso
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def cmd_vel_callback(self, msg):
        # Estrae i comandi di velocità inviati ai motori
        self.current_v_lin = msg.linear.x
        self.current_v_ang = msg.angular.z

    def scan_callback(self, msg):
        # Filtra i dati infiniti e trova l'ostacolo fisico più vicino
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.min_lidar_distance = min(valid_ranges)

    def camera_callback(self, msg):
        # Conferma che il flusso video è attivo registrando l'ora dell'ultimo frame
        sec = msg.header.stamp.sec
        self.last_camera_timestamp = f"Frame_{sec}"

    def keys_callback(self, msg):
        # Riceve la posizione della chiave dal maze_solver.py
        self.keys_logged = msg.data
        self.get_logger().info(f"LOG Salvato: {msg.data}")

    def save_data_to_csv(self):
        # Scrive la riga nel file CSV
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        self.csv_writer.writerow([
            current_time,
            f"{self.current_x:.3f}",
            f"{self.current_y:.3f}",
            f"{self.current_v_lin:.3f}",
            f"{self.current_v_ang:.3f}",
            f"{self.min_lidar_distance:.3f}",
            self.last_camera_timestamp,
            self.keys_logged
        ])
        
        # Pulisce l'evento chiave dopo averlo scritto, per non ripeterlo
        if self.keys_logged != "":
            self.keys_logged = ""

    def destroy_node(self):
        # Chiude il file in modo sicuro quando spegni ROS
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
