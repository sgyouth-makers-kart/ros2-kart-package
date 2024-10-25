import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WatergunservoNode(Node):
    def __init__(self):
        super().__init__('watergun_servo')
        self.subscription = self.create_subscription(
            String,
            'watergun_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received watergun value: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    watergun_servo_node = WatergunservoNode()
    rclpy.spin(watergun_servo_node)
    watergun_servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
