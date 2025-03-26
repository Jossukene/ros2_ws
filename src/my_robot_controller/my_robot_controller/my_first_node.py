# MET0310 - Autonomous Vehicle - Practice Tallinn University of Technology
# $ colcon build --symlink-install
# Step 4: Add a Node to the Package
# 1. Navigate to the package directory:
# $ cd ~/ros2_ws/src/my_robot_controller/my_robot_controller
# 2. Create a new Python file named my_first_node.py. You can do this in VS Code or the
# terminal:
# $ touch my_first_node.py
# 3. Add the following content to the file:
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node") # Node name
        self._counter = 0
        self.create_timer(1, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info(f"Hello {self._counter}")
        self._counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node) # Keep the node alive
    rclpy.shutdown()

if __name__ == '__main__':
    main()