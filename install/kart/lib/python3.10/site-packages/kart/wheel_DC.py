import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Motor
from time import sleep

class WheelDCNode(Node):
    def __init__(self):
        super().__init__('wheel_dc')
        self.subscription = self.create_subscription(
            String,
            'wheel_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # GPIO 핀 설정 (L298N의 IN1과 IN2를 GPIO 핀에 연결)
        forward_pin = 15 # GPIO22
        backward_pin = 16 # GPIO23
        self.motor = Motor(forward=forward_pin, backward=backward_pin)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received wheel value: {msg.data}')

        try:
            speed = float(msg.data)
            speed = max(-1, min(speed, 1)) #speed값 -1 ~ 1로 제한

            if speed >= 0:
                self.get_logger().info(f"Motor moving forward at {speed * 100}% speed")
                self.motor.forward(speed)
            else:
                speed = -speed
                self.get_logger().info(f"Motor moving backward at {speed * 100}% speed")
                self.motor.backward(speed)
        
        except ValueError:
            self.get_logger().error("tlqkf")

def main(args=None):
    rclpy.init(args=args)
    wheel_dc_node = WheelDCNode()
    rclpy.spin(wheel_dc_node)
    wheel_dc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
