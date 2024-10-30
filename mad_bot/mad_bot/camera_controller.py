from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import apriltag
import argparse

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        # Define a QoS profile
        qos_profile = QoSProfile(depth=10)
        
        # Create the subscription with the QoS profile
        # set up subscription to listen for Images from gazebo camera
        self.image_stream = self.create_subscription(Image, "/camera/image_raw", self.image_process, qos_profile)


    def image_process(self, msg):
        bridge = CvBridge()

        # try converting the iImage message to cv2 format
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

                # loop over the AprilTag detection results
        for r in self.results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv.line(self.image, ptA, ptB, (0, 255, 0), 2)
            cv.line(self.image, ptB, ptC, (0, 255, 0), 2)
            cv.line(self.image, ptC, ptD, (0, 255, 0), 2)
            cv.line(self.image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv.circle(self.image, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv.putText(self.image, tagFamily, (ptA[0], ptA[1] - 15),
                cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))
        # show the output image after AprilTag detection

        # reszie the image and display on screen
        resized_image = cv.resize(self.image, (640, 480))
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
