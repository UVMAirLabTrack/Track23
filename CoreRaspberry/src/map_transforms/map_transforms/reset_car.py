import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import select

class ResetCarPub(Node):
    def __init__(self):
        super().__init__('reset_car_node')
        self.publisher = self.create_publisher(Bool, 'reset_car', 10)
        self.reset_timer = None
        self.reset_active = False
        self.wait_time = 5.0

    def start_reset_timer(self):
        if not self.reset_active:
            self.reset_active = True
            msg = Bool()
            msg.data = True  # Publish True to activate reset
            self.publisher.publish(msg)
            self.reset_timer = self.create_timer(self.wait_time, self.reset_timer_callback)
            print(f"Reset activated, Timeout={self.wait_time}")

    def reset_timer_callback(self):
        if self.reset_active:
            self.reset_active = False
            msg = Bool()
            msg.data = False  # Publish False to deactivate reset
            self.publisher.publish(msg)
            if self.reset_timer:
                self.reset_timer.cancel()
                self.reset_timer = None
            print("Reset deactivated")

    def check_key_press(self):
        # Non-blocking check for keyboard input
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == '\n':  # Check for Enter key press
                self.start_reset_timer()
            print(f"Current reset state: {'active' if self.reset_active else 'inactive'}")

def main(args=None):
    print(f'To reset car, press Enter.')
    rclpy.init(args=args)
    reset_car_node = ResetCarPub()
    
    while rclpy.ok():
        reset_car_node.check_key_press()
        rclpy.spin_once(reset_car_node, timeout_sec=0.1)

    reset_car_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
