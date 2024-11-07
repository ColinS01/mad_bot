import os
from ament_index_python import get_package_share_directory
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, Bool
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import apriltag
import numpy as np
import glob

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        qos_profile = QoSProfile(depth=10)
        self.obj_height = 0
        self.obj_center = 0
        self.frame_center = 1280 // 2
        self.error = None

        self.image_stream = self.create_subscription(Image, "/camera/image_raw", self.image_process, qos_profile)
        self.offset_publisher = self.create_publisher(Int32, "/camera_error", qos_profile)
        self.distance_publisher = self.create_publisher(Float32, "/camera_distance", qos_profile)
        self.detect_publisher = self.create_publisher(Bool, '/camera_detection', 10)

        # Start calibration
        self.calibrate_camera()

    def calibrate_camera(self):
        chessboard_size = (7, 6)  # Number of inner corners in the chessboard
        square_size = 0.025  # Size of squares in meters

        objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane

        # Capture images for calibration
        images = glob.glob(os.path.join(get_package_share_directory('mad_bot'), 'image', '*.jpg'))

        for fname in images:
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                imgpoints.append(corners2)

                # Draw and display the corners
                cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
                cv.imshow('Calibration Image', img)
                cv.waitKey(500)  # Display for a short time

        cv.destroyAllWindows()

        # Perform calibration
        if len(objpoints) > 0 and len(imgpoints) > 0:
            ret, mtx, dist, _, _ = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

            if ret:
                np.savez('calibration_data.npz', mtx=mtx, dist=dist)
                self.get_logger().info("Camera calibrated successfully!")
            else:
                self.get_logger().error("Camera calibration failed.")
        else:
            self.get_logger().error("Not enough images for calibration.")

    def image_process(self, msg):
        bridge = CvBridge()
        self.obj_height = 0

        # Try converting the image message to cv2 format
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error with stream: {e}")
            return
        
        self.image = cv_image
        self.gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        self.gray = cv.GaussianBlur(self.gray, (5, 5), 0)
        self.options = apriltag.DetectorOptions(families="tag36h11", nthreads=4, refine_edges=True)
        self.detector = apriltag.Detector(self.options)
        self.results = self.detector.detect(self.gray)

        # Loop over the AprilTag detection results
        for r in self.results:
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
            self.obj_height = ptD[1] - ptA[1]

            # Find center of object for offset
            self.obj_center = ((ptC[0] - ptD[0]) / 2) + ptD[0]

            # If the height is existing then an object is detected
            if self.obj_height > 0:
                distance = self.calculate_distance()
                self.calculate_offset()
                cv.putText(self.image, str(distance), (ptA[0], ptA[1] - 15),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
        if self.obj_height == 0:
            detect_msg = Bool()
            detect_msg.data = False
            self.detect_publisher.publish(detect_msg)


        # Show the output image after AprilTag detection
        resized_image = cv.resize(self.image, (640, 480))
        cv.imshow("camera_stream", resized_image)
        cv.waitKey(1)

    def calculate_distance(self):
        F = 1.0
        H = 200
        d = (F * H) / self.obj_height
        msg = Float32()
        msg.data = d
        self.distance_publisher.publish(msg)
        return d

    def calculate_offset(self):
        self.error = float(self.frame_center - self.obj_center)
        msg = Int32()
        msg.data = int(self.error)
        self.offset_publisher.publish(msg)

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
