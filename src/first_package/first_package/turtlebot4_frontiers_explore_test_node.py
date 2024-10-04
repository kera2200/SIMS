import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid


class TurtleBot4FrontiersExploreTestNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_frontiers_explore_test_node')
        self.get_logger().info('TurtleBot4FrontiersExploreTestNode has been started.')
        
        self.frontiers = []
        

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/turtle/map',
            self.map_callback,
            QoSProfile(depth=10))
        
        
    def map_callback(self, msg : OccupancyGrid):
        self.get_logger().info(f"Map received) {msg.info.width}x{msg.info.height}")
        self.frontiers = self.detect_frontiers(msg)

        coordinate = self.get_frontier_coordinate(random.choice(self.frontiers))

        self.get_logger().info(f"Frontier coordinate: {coordinate}")


    def detect_frontiers(self, msg: OccupancyGrid):
        self.get_logger().info(f"Detecting frontiers")
        frontiers = []

        width = msg.info.width
        height = msg.info.height
        data = msg.data

        for i in range(len(data)):
            if self.is_frontier(i, width, height, data):
                frontiers.append(i)

        return frontiers

    def is_frontier(self, index, width, height, data):
        #self.get_logger().info(f"Checking if frontier")
        # Check if the cell is unknown
        if data[index] != -1:
            return False

        # Calculate row and column of the cell
        row = index // width
        col = index % width

        # Define the 4 possible neighbors (row, col) offsets
        neighbors = [(row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)]

        # Check each neighbor
        for neighbor_row, neighbor_col in neighbors:
            # Check if neighbor is within bounds
            if 0 <= neighbor_row < height and 0 <= neighbor_col < width:
                neighbor_index = neighbor_row * width + neighbor_col
                # Check if neighbor is free space
                if data[neighbor_index] == 0:
                    return True

        return False
    
    def get_frontier_coordinate(self, index):

        frontier = self.frontiers[index]

        # Get the map information
        width = self.map.info.width
        resolution = self.map.info.resolution
        origin = self.map.info.origin

        # Convert the index to row and column
        row = frontier // width
        col = frontier % width 

        # Calculate the x and y coordinates
        x = col * resolution + origin.position.x
        y = row * resolution + origin.position.y

        return (x, y)

        


        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FrontiersExploreTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        