import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import cv2 as cv
import os
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        self.template_path = os.path.join(get_package_share_directory('mad_bot'), 'image', 'file.jpg')
        self.load_template()
        self.cap = cv.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()
            return
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Vector3, 'tracked_position', qos_profile)
        self.tracking = False
        self.timer = self.create_timer(0.1, self.capture_frames)
        self.create_subscription(String, '/keyboard_input', self.key_callback, 10)

    def load_template(self):
        template = cv.imread(self.template_path)
        if template is None:
            self.get_logger().error("Template image not found.")
            rclpy.shutdown()
            return
        hsv_template = cv.cvtColor(template, cv.COLOR_BGR2HSV)
        average_hsv = cv.mean(hsv_template)[:3]
        hue, saturation, value = average_hsv
        self.lower_bound = np.array([hue - 10, saturation - 50, value - 50])
        self.upper_bound = np.array([hue + 10, saturation + 50, value + 50])

    def capture_frames(self):
        ret, self.frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame")
            return
        hsv_frame = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_frame, self.lower_bound, self.upper_bound)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        main_object_position = None

        for contour in contours:
            if cv.contourArea(contour) > 500:
                x, y, w, h = cv.boundingRect(contour)
                main_object_position = (x + w // 2, y + h // 2)
                cv.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                break

        if self.tracking and main_object_position:
            position_msg = Vector3()
            position_msg.x = float(main_object_position[0])
            position_msg.y = float(main_object_position[1])
            position_msg.z = 0.0
            self.publisher.publish(position_msg)
            self.get_logger().info(f"Position Captured: {position_msg.x}, {position_msg.y}")

        cv.imshow('Camera Feed', self.frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.tracking = False

    def key_callback(self, msg):
        if msg.data == "space":
            self.tracking = not self.tracking
            if self.tracking:
                self.get_logger().info("Started tracking.")
            else:
                self.get_logger().info("Stopped tracking.")

def main(args=None):
    rclpy.init(args=args)
    color_tracker = ColorTracker()
    
    try:
        rclpy.spin(color_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        color_tracker.cap.release()
        cv.destroyAllWindows()
        color_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
