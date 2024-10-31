from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import apriltag

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        # Define a QoS profile
        qos_profile = QoSProfile(depth=10)
        self.obj_height = 0
        self.obj_center = 0
        self.frame_center = 640 // 2
        self.error = None
        
        # Create the subscription with the QoS profile
        self.image_stream = self.create_subscription(Image, "/camera/image_raw", self.image_process, qos_profile)
        self.publisher = self.create_publisher(Int32, "/camera_error", qos_profile)

    def image_process(self, msg):
        bridge = CvBridge()

        # Try converting the image message to cv2 format
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error with stream: {e}")
            return
        
        self.image = cv_image
        self.gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self.results = self.detector.detect(self.gray)

        # Loop over the AprilTag detection results
        for r in self.results:
            # Extract the bounding box coordinates for the AprilTag
            (ptA, ptB, ptC, ptD) = r.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            # Draw the bounding box of the AprilTag detection
            cv.line(self.image, ptA, ptB, (0, 255, 0), 2)
            cv.line(self.image, ptB, ptC, (0, 255, 0), 2)
            cv.line(self.image, ptC, ptD, (0, 255, 0), 2)
            cv.line(self.image, ptD, ptA, (0, 255, 0), 2)

            # Draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv.circle(self.image, (cX, cY), 5, (0, 0, 255), -1)

            # Calculate height in pixels
            self.obj_height = ptD[1] - ptA[1]  # y-coordinates of top and bottom points
            self.obj_center = ((ptC[0] - ptD[0]) / 2) + ptD[0]
            self.get_logger().info(str(self.obj_height))
            if self.obj_height > 0:
                distance = self.calculate_distance()
                self.calculate_offset()
                cv.putText(self.image, str(distance), (ptA[0], ptA[1] - 15),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the output image after AprilTag detection
        resized_image = cv.resize(self.image, (640, 480))
        cv.imshow("camera_stream", resized_image)
        cv.waitKey(1)  # Ensure the image window updates

    def calculate_distance(self):
        F = 0.5
        H = 200
        d = (F * H) / self.obj_height
        return d

    def calculate_offset(self):
        self.error = float(self.frame_center - self.obj_center)
        msg = Int32()
        msg.data = int(self.error)  # Correct the variable name
        self.get_logger().info(f'Error: {self.error}')  # Proper logging
        self.publisher.publish(msg)

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
