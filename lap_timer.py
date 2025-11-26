import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer_node')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.listener_callback,
            10)
            
        self.start_x = None
        self.start_y = None
        self.last_time = time.time()
        
        self.left_start_zone = False
        self.start_radius = 2.0
        
        self.lap_count = 0
        self.best_lap = float('inf')

        print("\n" + "="*40)
        print("   CRONÓMETRO DE VUELTAS F1TENTH   ")
        print("   Esperando movimiento...         ")
        print("="*40 + "\n")

    def listener_callback(self, msg):
        pos = msg.pose.pose.position
        
        if self.start_x is None:
            self.start_x = pos.x
            self.start_y = pos.y
            self.last_time = time.time()
            return

        dist_to_start = math.sqrt((pos.x - self.start_x)**2 + (pos.y - self.start_y)**2)

        if dist_to_start > self.start_radius:
            self.left_start_zone = True

        if self.left_start_zone and dist_to_start < 1.0:
            current_time = time.time()
            lap_duration = current_time - self.last_time
            
            self.lap_count += 1
            is_best = ""
            if lap_duration < self.best_lap:
                self.best_lap = lap_duration
                is_best = "¡¡NUEVO RÉCORD!! 🏆"

            self.get_logger().info(f'🏁 VUELTA {self.lap_count}: {lap_duration:.3f} seg  {is_best}')
            
            self.last_time = current_time
            self.left_start_zone = False

def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
