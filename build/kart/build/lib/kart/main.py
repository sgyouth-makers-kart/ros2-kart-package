import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO


SHOOT_SIZE = 10000
CAMERA_LEFT = 210
CAMERA_RIGHT = 430
WATERGUN_ANGLE_RATIO = 480


class MainNode(Node):
    def __init__(self):
        super().__init__('main')
        self.topics = ['pump', 'camera', 'steering', 'watergun', 'wheel']
        self._publishers = {topic: self.create_publisher(String, f'{topic}_value', 10) for topic in self.topics}

    def publish_value(self, topic, value):
        msg = String()
        msg.data = value
        self._publishers[topic].publish(msg)
        self.get_logger().info(f'Publishing to {topic}: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    cap = cv2.VideoCapture(0)
    model = YOLO("yolo11n.pt")

    while rclpy.ok():
        ret, frame = cap.read()

        if not ret:
            print("VideoCapture failed")
            break

        max_size = 0
        nearest_person = None

        results = model(frame)
        result = results[0]
        if len(result.summary()) > 0:
            for item in result.summary():
                if item["name"] == "person":
                    box = item["box"]
                    size = (box["x2"] - box["x1"]) * (box["y2"] - box["y1"])
                    if size > max_size:
                        max_size = size
                        nearest_person = ((box["x2"] - box["x1"]) / 2, (box["y2"] - box["y1"] / 2))

                    # Draw box
                    cv2.rectangle(frame, (int(box["x1"]), int(box["y1"])), (int(box["x2"]), int(box["y2"])), (0, 255, 0), 2)
        else:
            print("Nothing detected")

        # wheel_value = "0"
        # steering_value = "0"
        # camera_value = "0"
        # pump_value = "0"
        # watergun_value = "0"

        if nearest_person:
            if max_size < SHOOT_SIZE:
                wheel_value = "1"
            else:
                wheel_value = "0"

            if nearest_person[0] < CAMERA_LEFT:
                camera_value = steering_value = "-1"
            elif nearest_person[0] < CAMERA_RIGHT:
                camera_value = steering_value = "0"
                if wheel_value == "0":
                    pump_value = "1"
            else:
                camera_value = steering_value = "1"

            watergun_value = str(nearest_person[1] / WATERGUN_ANGLE_RATIO)
        else:
            steering_value = "1"
            wheel_value = "1"

        if nearest_person:
            print(f"nearest person: {nearest_person}, max_size: {max_size}")
            pump_value = "1"


        node.publish_value("pump", pump_value)
        node.publish_value("camera", camera_value)
        node.publish_value("steering", steering_value)
        node.publish_value("watergun", watergun_value)
        node.publish_value("wheel", wheel_value)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
