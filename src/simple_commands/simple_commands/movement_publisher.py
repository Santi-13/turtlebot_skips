import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist

class MovementPublisher(Node):
    def __init__(self):
        super().__init__('movement_publisher')
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos_profile )
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x_vel = 0.0
        self.y_vel = 0.0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.x_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, linear.y={msg.linear.y}')
        self.x_vel = (self.x_vel + 0.01) % 0.2
        self.y_vel = (self.y_vel + 0.01) % 0.2

def main(args=None):
    rclpy.init(args=args)
    movement_publisher = MovementPublisher()
    rclpy.spin(movement_publisher)
    movement_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
