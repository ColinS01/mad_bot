import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import tkinter as tk

class AprilTagController(Node):
    def __init__(self):
        super().__init__('april_tag_controller')
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Initial model state
        self.model_state = EntityState()
        self.model_state.name = 'april_tag'  # Replace with your actual entity name
        self.model_state.pose.position.x = 0.0
        self.model_state.pose.position.y = 0.0
        self.model_state.pose.position.z = 0.0
        self.model_state.pose.orientation.x = 0.0
        self.model_state.pose.orientation.y = 0.0
        self.model_state.pose.orientation.z = 0.0
        self.model_state.pose.orientation.w = 1.0

        self.master = tk.Tk()
        self.master.title("April Tag Controller")
        self.master.bind("<KeyPress>", self.move_tag)
        
        self.update_ros()

    def move_tag(self, event):
        # Move increment value
        increment = 0.1
        
        if event.char.lower() == "a":  # Move left
            self.model_state.pose.position.y += increment
        elif event.char.lower() == "d":  # Move right
            self.model_state.pose.position.y -= increment
        elif event.char.lower() == "w":  # Move up
            self.model_state.pose.position.x += increment
        elif event.char.lower() == "s":  # Move down
            self.model_state.pose.position.x -= increment
        
        # Call the service to set the new entity state
        self.set_entity_state()

    def set_entity_state(self):
        req = SetEntityState.Request()
        req.entity_name = self.model_state.name
        req.pose = self.model_state.pose  # Directly use the pose from model_state
        self.cli.call_async(req)

    def update_ros(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.master.after(100, self.update_ros)

def main(args=None):
    rclpy.init(args=args)
    april_tag_controller = AprilTagController()

    try:
        april_tag_controller.master.mainloop()
    finally:
        april_tag_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
