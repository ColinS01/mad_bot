import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Int32, '/camera_error', self.move_robot, 10)
        self.angular_vel = 0.0
        self.linear_vel = 0.0
        
        # proportional control
        self.kp = 0.007  
        self.linear_speed = 0.6

    def move_robot(self, msg):
        error = msg.data
        
        # Calculate angular velocity using a proportional controller
        self.angular_vel = self.kp * error  

        # Check if error is within the range to apply linear velocity
        if -300 < error < 300:
            self.linear_vel = self.linear_speed
        else:
            self.linear_vel = 0.0  # Stop linear movement if out of range
        
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.angular.z = self.angular_vel
        
        self.publisher.publish(twist_msg)


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
