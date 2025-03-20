import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class TurtlePathController(Node):
    def __init__(self):
        super().__init__('turtle_path_controller')
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.current_path = 'circle'
        self.step = 0
        self.start_time = time.time()

    def control_loop(self):
        elapsed_time = time.time() - self.start_time
        if self.current_path == 'circle':
            self.move_in_circle()
            if elapsed_time > 10:  # Switch path after 10 seconds
                self.switch_path('square')

        elif self.current_path == 'square':
            self.move_in_square()
            if self.step >= 4:  # Square has 4 sides
                self.switch_path('triangle')

        elif self.current_path == 'triangle':
            self.move_in_triangle()
            if self.step >= 3:  # Triangle has 3 sides
                self.switch_path('circle')

    def move_in_circle(self):
        msg = Twist()
        msg.linear.x = 2.0  # Constant forward speed
        msg.angular.z = 1.0  # Constant angular speed to form a circle
        self.publisher.publish(msg)

    def move_in_square(self):
        if self.step % 2 == 0:
            # Move forward for a set duration
            msg = Twist()
            msg.linear.x = 2.0
            self.publisher.publish(msg)
            time.sleep(1)  # Move forward for 1 second
        else:
            # Turn 90 degrees
            msg = Twist()
            msg.angular.z = math.pi / 2  # 90-degree turn
            self.publisher.publish(msg)
            time.sleep(0.5)  # Duration for the turn
        self.step += 1

    def move_in_triangle(self):
        if self.step % 2 == 0:
            # Move forward for a set duration
            msg = Twist()
            msg.linear.x = 2.0
            self.publisher.publish(msg)
            time.sleep(1)  # Move forward for 1 second
        else:
            # Turn 120 degrees
            msg = Twist()
            msg.angular.z = 2 * math.pi / 3  # 120-degree turn
            self.publisher.publish(msg)
            time.sleep(0.6)  # Duration for the turn
        self.step += 1

    def switch_path(self, new_path):
        self.get_logger().info(f'Switching path to {new_path}')
        self.current_path = new_path
        self.step = 0
        self.start_time = time.time()  # Reset time for the new path

def main(args=None):
    rclpy.init(args=args)
    controller = TurtlePathController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
