import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class FTGSenna(Node):
    def __init__(self):
        super().__init__('ftg_senna_node')

        # --- MODO SENNA / TURBO ---
        self.MAX_SPEED = 11.0       # Velocidad punta
        self.CORNER_SPEED = 3.5     # Velocidad en curva
        self.BRAKE_DIST = 4.0       # Distancia de frenado
        self.ACCEL_RATE = 1.0       # Aceleración inmediata
        self.STRAIGHT_TOLERANCE = 0.08  # Tolerancia para considerar recta
        
        # ESTABILIDAD
        self.BUBBLE_RADIUS = 25     
        self.FOV_CROP = 200         

        self.current_speed = 0.0
        
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.get_logger().info('--- MODO SENNA ACTIVADO: 11.0 m/s ---')

    def preprocess_lidar(self, ranges):
        proc_ranges = np.array(ranges)
        proc_ranges[np.isinf(proc_ranges)] = 10.0
        proc_ranges[np.isnan(proc_ranges)] = 0.0
        proc_ranges[:self.FOV_CROP] = 0.0
        proc_ranges[-self.FOV_CROP:] = 0.0
        return proc_ranges

    def find_best_point(self, start_i, end_i, ranges):
        return (start_i + end_i) // 2

    def calculate_turbo_speed(self, steering_angle, front_dist):
        if abs(steering_angle) < self.STRAIGHT_TOLERANCE:
            target_speed = self.MAX_SPEED
        else:
            steering_penalty = abs(steering_angle) * 14.0 
            target_speed = self.MAX_SPEED - steering_penalty
        
        target_speed = max(self.CORNER_SPEED, target_speed)

        if front_dist < self.BRAKE_DIST:
            brake_factor = (front_dist / self.BRAKE_DIST) ** 0.8
            braking_speed = self.MAX_SPEED * brake_factor
            target_speed = min(target_speed, braking_speed)
            target_speed = max(self.CORNER_SPEED, target_speed)

        return target_speed

    def scan_callback(self, msg):
        ranges = self.preprocess_lidar(msg.ranges)
        
        masked_ranges = np.where(ranges == 0.0, 100.0, ranges)
        closest_idx = np.argmin(masked_ranges)
        if ranges[closest_idx] < 0.25:
            self.publish_drive(0.0, 0.0)
            return

        start_b = max(0, closest_idx - self.BUBBLE_RADIUS)
        end_b = min(len(ranges), closest_idx + self.BUBBLE_RADIUS)
        ranges[start_b:end_b] = 0.0

        threshold = 2.5
        mask = ranges > threshold
        change_indices = np.where(np.diff(np.concatenate(([False], mask, [False]))))[0]
        best_gap = (0, 0)
        max_len = 0
        for i in range(0, len(change_indices), 2):
            l = change_indices[i+1] - change_indices[i]
            if l > max_len: max_len = l; best_gap = (change_indices[i], change_indices[i+1])

        if max_len == 0:
            steering_angle = 0.0
        else:
            target_idx = self.find_best_point(best_gap[0], best_gap[1], ranges)
            steering_angle = msg.angle_min + (target_idx * msg.angle_increment)

        center_idx = len(ranges) // 2
        front_dist = np.mean(masked_ranges[center_idx-5 : center_idx+5])
        
        target_speed = self.calculate_turbo_speed(steering_angle, front_dist)

        if target_speed > self.current_speed:
            self.current_speed = min(target_speed, self.current_speed + self.ACCEL_RATE)
        else:
            self.current_speed = target_speed

        self.publish_drive(self.current_speed, steering_angle)

    def publish_drive(self, speed, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(angle)
        self.publisher_drive.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FTGSenna()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
