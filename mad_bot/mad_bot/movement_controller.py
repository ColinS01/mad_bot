import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Int32, '/camera_error', self.adjust_angle, 10)
        self.angular_vel = 0.0
        self.linear_vel = 0.0
        
        # Set a gain for the proportional control
        self.kp = 0.01  # Adjust this value as necessary

    def adjust_angle(self, msg):
        # Extract the error from the message
        error = msg.data
        
        # Calculate angular velocity using a proportional controller
        self.angular_vel = self.kp * error  # Negative sign to turn in the correct direction
        
        # Create a Twist message to publish
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel  # You can set this to a desired value
        twist_msg.angular.z = self.angular_vel
        
        # Publish the Twist message
        self.publisher.publish(twist_msg)

    def move_toward_target(self):
        # Implement movement towards target if needed
        pass

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
