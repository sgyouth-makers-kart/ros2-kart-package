import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Motor
from time import sleep

class PumpDCNode(Node):
    def __init__(self):
        super().__init__('pump_dc')
        self.subscription = self.create_subscription(
            String,
            'pump_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # GPIO 핀 설정 (L298N의 IN1과 IN2를 GPIO 핀에 연결)
        forward_pin = 17
        backward_pin = 27
        self.motor = Motor(forward=forward_pin, backward=backward_pin)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received pump value: {msg.data}')
        
        try:
            speed = float(msg.data)
            speed = max(0, min(speed, 1)) #speed값 0 ~ 1로 제한

            self.get_logger().info(f"pump working at {speed * 100}% power")
            self.motor.forward(speed)

        except ValueError:
            self.get_logger().error("tlqkf")

def main(args=None):
    rclpy.init(args=args)
    pump_dc_node = PumpDCNode()
    rclpy.spin(pump_dc_node)
    pump_dc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

