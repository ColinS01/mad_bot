from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        
        # Define a QoS profile
        qos_profile = QoSProfile(depth=10)
        
        # Create the subscription with the QoS profile
        self.image_stream = self.create_subscription(Image, "/camera/image_raw", self.image_process, qos_profile)

    def image_process(self, msg):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error with stream: {e}")
            return
        
        image = cv_image

        resized_image = cv.resize(image, (640, 480))
        cv.imshow("camera_stream", resized_image)
        cv.waitKey(1)  # Add this line to ensure the image window updates

def main(args=None):
    rclpy.init(args=args)
    camera_controller = CameraController()

    try:
        rclpy.spin(camera_controller)
    finally:
        camera_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
