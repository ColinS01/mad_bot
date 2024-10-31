import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Int32, '/camera_error', self.move_robot, 10)
        self.distance_sub = self.create_subscription(Float32, '/camera_distance', self.apply_linear, 10)
        
        self.angular_vel = 0.0
        self.linear_vel = 0.0
        self.last_error = 0  # Initialize to 0 for the first comparison
        
        # Proportional control parameters
        self.kp = 0.007  
        self.linear_speed = 1.2
        self.smoother = 0.003
        self.bonus_accel = 0.4
        self.max_angular_vel = 1.0  # Cap for angular velocity
        self.max_linear_vel = 2.0  # Cap for linear velocity

    def move_robot(self, msg):
        error = msg.data
        
        # Determine if the object is moving left or right
        if self.last_error is not None:
            if error > self.last_error:
                # Object moving away from center (right)
                self.angular_vel -= self.smoother
            elif error < self.last_error:
                # Object moving toward center (left)
                self.angular_vel += self.smoother
        
        # Calculate angular velocity using a proportional controller
        self.angular_vel = self.kp * error  
        
        # Check if error is within the range to apply linear velocity
        if -200 < error < 200:
            self.linear_vel = self.linear_speed
        elif abs(error) > 300:
            self.linear_vel = 0.0  # Stop linear movement if out of range
        else:
            self.linear_vel += self.bonus_accel

        # Cap the velocities
        self.angular_vel = max(min(self.angular_vel, self.max_angular_vel), -self.max_angular_vel)
        self.linear_vel = min(self.linear_vel, self.max_linear_vel)

        # Update last_error for the next iteration
        self.last_error = error
        
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.angular.z = self.angular_vel
        
        self.publisher.publish(twist_msg)

    def apply_linear(self, msg):
        distance = msg.data
        if distance > 2:
            self.linear_speed += 0.8
        else:
            self.linear_speed = 0.9


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
