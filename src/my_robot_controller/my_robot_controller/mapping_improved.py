import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

        # Subscriber to receive the map
        self._map_listener = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # Map data will be stored here
        self.map_data = None

    def map_callback(self, msg: OccupancyGrid):
        # Store the latest map data
        self.map_data = msg

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Define the width of the range for obstacle detection
        a = 5

        # Extract directional distances
        self._front = min(scan.ranges[:a+1] + scan.ranges[-a:])
        self._left = min(scan.ranges[90-a:90+a+1])
        self._right = min(scan.ranges[270-a:270+a+1])

        # Handle the case when the robot detects being stuck in a narrow corridor
        if self._front < 0.3 and self._left < 0.3 and self._right < 0.3:  # Robot stuck in a narrow corridor
            self.get_logger().info("Robot is stuck in a narrow corridor. Reversing...")
            # Reverse and try to free the robot from a narrow space
            cmd.linear.x = -0.1  # Reverse
            cmd.angular.z = 0.0  # No turning
        elif self._front < 1.0:  # Obstacle detected in front
            # In a narrow space, we need to be more cautious, turn slowly
            if self._right < self._left:
                cmd.linear.x = 0.0  # Stop moving forward
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.linear.x = 0.0  # Stop moving forward
                cmd.angular.z = -0.5  # Turn right
        elif self._front < 2.0:  # Obstacle is further but still near
            # Slow down as the robot approaches the obstacle
            cmd.linear.x = 0.2  # Slow forward speed
            cmd.angular.z = 0.0  # Move straight
        else:
            # Move forward when no obstacle is detected
            cmd.linear.x = 0.3  # Normal speed
            cmd.angular.z = 0.0  # No turning

        # Handle case of narrow corridors dynamically
        if self._left < 0.5 and self._right < 0.5:  # Too narrow to move
            self.get_logger().info("Narrow corridor detected. Slowing down.")
            # Slow down further and stop or turn slowly
            cmd.linear.x = 0.1  # Slow movement
            cmd.angular.z = 0.2  # Turn slightly to navigate

        # If the map is available, perform frontier exploration
        if self.map_data:
            frontier = self.detect_frontier()

            if frontier:
                # Move towards the frontier
                cmd = self.move_towards_frontier(frontier)

        # Publish the command
        self._pose_publisher.publish(cmd)

    def detect_frontier(self):
        # Get the map data as a numpy array
        if not self.map_data:
            return None

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        # Create a 2D grid from the occupancy grid data
        grid = np.array(self.map_data.data).reshape((height, width))

        # Frontier detection: Look for borders between free space (0) and unknown space (-1)
        frontier_points = []

        for y in range(height):
            for x in range(width):
                # Check for a border between free (0) and unknown (-1) space
                if grid[y, x] == 0:  # Free space
                    if self.is_frontier(x, y, grid, width, height):
                        frontier_points.append((x, y))

        # Return the closest frontier point if found
        if frontier_points:
            return frontier_points[0]  # For simplicity, we choose the first frontier found
        else:
            return None

    def is_frontier(self, x, y, grid, width, height):
        # Check if the neighboring cells are unknown (indicating a frontier)
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if grid[ny, nx] == -1:  # Unknown space
                        return True
        return False

    def move_towards_frontier(self, frontier):
        cmd = Twist()

        # Convert map coordinates (x, y) to robot's coordinate system
        frontier_x, frontier_y = frontier
        robot_x = self.map_data.info.origin.position.x
        robot_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution

        # Convert grid coordinates to meters
        frontier_x = frontier_x * resolution + robot_x
        frontier_y = frontier_y * resolution + robot_y

        # Calculate the direction to the frontier
        angle_to_frontier = math.atan2(frontier_y - robot_y, frontier_x - robot_x)
        distance_to_frontier = math.sqrt((frontier_x - robot_x) ** 2 + (frontier_y - robot_y) ** 2)

        # Move towards the frontier
        cmd.linear.x = min(0.2, distance_to_frontier)  # Move forward with a limited speed
        cmd.angular.z = 0.5 * angle_to_frontier  # Turn toward the frontier

        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
