import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import tkinter as tk

class AprilTagController(Node):
    def __init__(self):
        super().__init__('april_tag_controller')
        self.publisher = self.create_publisher(Vector3, '/april_pos', 10)
        self.pos = (0, 0, 0)
        self.master = tk.Tk()
        self.master.title("April Tag Controller")
        self.master.bind("<KeyPress>", self.move_tag)
        
        self.update_ros()

    def move_tag(self, event):
        x = 0.0
        y = 0.0
        
        if event.char.lower() == "a":
            y = 1.0
        elif event.char.lower() == "d":
            y = -1.0
        if event.char.lower() == "w":
            x = 1.0
        elif event.char.lower() == "s":
            x = -1.0
        
        if x != 0.0 or y != 0.0:
            msg = Vector3()
            msg.x = x
            msg.y = y
            msg.z = 0.0
            self.publisher.publish(msg)

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
