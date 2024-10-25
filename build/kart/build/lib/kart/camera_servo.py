import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraservoNode(Node):
    def __init__(self):
        super().__init__('camera_servo')
        self.subscription = self.create_subscription(
            String,
            'camera_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received camera value: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    camera_dc_node = CameraservoNode()
    rclpy.spin(camera_dc_node)
    camera_dc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
