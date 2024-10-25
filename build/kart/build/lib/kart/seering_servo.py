import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Servo

# GPIO 14번 핀에 서보 모터 연결
servo = Servo(2)  # PiGPIOFactory 사용하지 않음

class SteeringServoNode(Node):
    def __init__(self):
        super().__init__('steering_servo')
        self.subscription = self.create_subscription(
            String,
            'steering_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            # 수신한 메시지를 각도로 변환
            angle = float(msg.data)
            if 0 <= angle <= 180:
                # 각도를 서보 모터의 위치 값으로 변환
                value = angle_to_value(angle)
                servo.value = value
                self.get_logger().info(f'Servo set to {angle} degrees')
            else:
                self.get_logger().warn('Received value out of range (0-180)')
        except ValueError:
            self.get_logger().error('Invalid steering value received')

def angle_to_value(angle):
    # 각도를 -1 ~ 1 범위로 변환
    MIN_ANGLE = 0    # 최소 각도
    MAX_ANGLE = 180  # 최대 각도
    MIN_SERVO = -1   # 서보의 최소 위치 (-1)
    MAX_SERVO = 1    # 서보의 최대 위치 (1)
    
    return (angle - MIN_ANGLE) * (MAX_SERVO - MIN_SERVO) / (MAX_ANGLE - MIN_ANGLE) + MIN_SERVO

def main(args=None):
    rclpy.init(args=args)
    steering_servo_node = SteeringServoNode()
    try:
        rclpy.spin(steering_servo_node)
    except KeyboardInterrupt:
        pass
    finally:
        steering_servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

