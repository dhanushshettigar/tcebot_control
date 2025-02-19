import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

MOVE_BINDINGS = {
    'w': (1, 0),  
    's': (-1, 0),  
    'a': (0, 1),  
    'd': (0, -1),
}

SPEED_BINDINGS = {
    '+': 1.1,
    '-': 0.9,
}

STOP_KEY = 'x'
EXIT_KEY = 'q'

CONTROL_MESSAGE = """
Control Your Robot:
---------------------------
Move:    
    W    
  A   D  
    S    

Stop:     X  
Speed Up: +  
Slow Down: -  
Exit:     Q  

"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.2
        self.turn = 1.0
        self.get_logger().info(CONTROL_MESSAGE)

        # Check if running in a terminal (TTY)
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.get_logger().error("❌ This script must be run in a terminal (TTY). Launch files are not supported.")
            rclpy.shutdown()

    def get_key(self):
        if not sys.stdin.isatty():
            return ''  # Return nothing if not a real terminal
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()

                if key in MOVE_BINDINGS:
                    twist.linear.x = self.speed * MOVE_BINDINGS[key][0]
                    twist.angular.z = self.turn * MOVE_BINDINGS[key][1]
                elif key in SPEED_BINDINGS:
                    self.speed *= SPEED_BINDINGS[key]
                    self.turn *= SPEED_BINDINGS[key]
                    self.get_logger().info(f"Speed: {self.speed:.2f}, Turn: {self.turn:.2f}")
                elif key == STOP_KEY:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("🚀 Robot Stopped! Press 'WASD' to move again.")
                elif key == EXIT_KEY:
                    self.get_logger().info("Exiting Teleop Keyboard... Goodbye! 👋")
                    break

                self.publisher_.publish(twist)
        finally:
            self.restore_terminal()

    def restore_terminal(self):
        if sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info("✅ Terminal settings restored.")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
