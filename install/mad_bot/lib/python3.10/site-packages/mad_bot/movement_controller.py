import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import math

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Vector3, '/tracked_position', self.tracked_position_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.phi = 0
        self.current_pos = (0, 0, 0)
        self.target_pos = None
        self.T = 10 

       
        self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):

        self.current_pos = (msg.pose.pose.position.x, 
                            msg.pose.pose.position.y, 
                            msg.pose.pose.position.z)
        
        orientation = msg.pose.pose.orientation
        self.phi = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info(f"Updated odometry: Position: {self.current_pos}, Yaw: {math.degrees(self.phi)} degrees")

    def quaternion_to_euler(self, w, x, y, z):
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return yaw


    def calculate_distance(self):
        if self.target_pos is not None:
            delta_x = self.target_pos[0] - self.current_pos[0]
            delta_z = self.target_pos[2] - self.current_pos[2]
            return np.sqrt(delta_x**2 + delta_z**2)
        return 0


    def calculate_angle(self):
        if self.target_pos is not None:
            delta_x = self.target_pos[0] - self.current_pos[0]
            delta_z = self.target_pos[2] - self.current_pos[2]
            return math.atan2(delta_z, delta_x) - self.phi
        return 0

    def calculate_linear_velocity(self, distance):
        if distance > 0:
            return distance / self.T
        return 0

    def calculate_angular_velocity(self, delta_angle):
        if delta_angle != 0:
            return delta_angle / self.T
        return 0

    def publish_twist(self, linear_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Published Twist message: Linear vel: {linear_vel}, Angular vel: {angular_vel}")

    def tracked_position_callback(self, msg):
        self.target_pos = (msg.x, 0, msg.z)
        self.get_logger().info(f"New target position set: {self.target_pos}")

    def timer_callback(self):
        if self.target_pos is not None:
            distance = self.calculate_distance()
            delta_angle = self.calculate_angle()
            linear_vel = self.calculate_linear_velocity(distance)
            angular_vel = self.calculate_angular_velocity(delta_angle)

            self.publish_twist(linear_vel, angular_vel)
        else:
            self.get_logger().info("Waiting for valid target position.")

def main(args=None):
    rclpy.init(args=args)
    movement_controller = MovementController()

    try:
        rclpy.spin(movement_controller)
    except KeyboardInterrupt:
        pass
    finally:
        movement_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
