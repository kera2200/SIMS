import rclpy
import random
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid

class TurtleBot4_Frontier_Explorer(Node):
    def __init__(self):
        super().__init__('turtlebot4_test')
        self.get_logger().info('TurtleBot4Test has been started.')

        self.action_client = ActionClient(self, NavigateToPose, '/turtle/navigate_to_pose')

        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/turtle/map',
            self.map_callback,
            10)
        
        self.map_data = None
        self.map_resolution = 0.0
        self.map_origin = 0.0
        self.width = 0
        self.height = 0
        self.is_navigating = False
        
    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.width = msg.info.width
        self.height = msg.info.height
        
        
    def send_goal(self):

        if self.map_data is None:
            self.get_logger().info('Map data is not available yet.')
            return
        self.get_logger().info('Sending goal to the action server.')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.random_valid_pose()

        self.action_client.wait_for_server()
        self.get_logger().info('Action server is available.')
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def random_valid_pose(self):
        frontiers_cells = self.frontiers_cells()

        if not frontiers_cells:
            self.get_logger().info('No free cells available.')
            return None
        
        random_free_cell = random.choice(frontiers_cells)
        x_index, y_index = random_free_cell
        x_world = self.map_origin.x + x_index * self.map_resolution
        y_world = self.map_origin.y + y_index * self.map_resolution

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x_world
        pose.pose.position.y = y_world
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Random frontier pose: {pose}')

        return pose


    def frontiers_cells(self):
        frontiers_cells = []
        for y in range(self.height):
            for x in range(self.width):
                index = y * self.width + x # 2D to 1D
                if self.map_data[index] == -1:
                    if self.is_frontier_cell(x, y):
                        frontiers_cells.append((x, y))
        return frontiers_cells
    
    def is_frontier_cell(self, x, y):
        # Check if the cell is within the map boundaries
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        # Check if the cell is occupied
        has_unknown_neighbor = False
        has_free_neighbor = False

        # Define the 8 possible neighbors (N, S, E, W, NE, NW, SE, SW)
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dx, dy in neighbors:
            # Calculate the neighbor's coordinates
            nx = x + dx
            ny = y + dy

            # Check if the neighbor is within the map boundaries
            if nx >= 0 and nx < self.width and ny >= 0 and ny < self.height:
                neighbor_index = ny * self.width + nx
                # Check if the neighbor is occupied
                if self.map_data[neighbor_index] == 0:
                    has_free_neighbor = True
                elif self.map_data[neighbor_index] == -1:
                    has_unknown_neighbor = True

            # If both conditions are met, return True
            if has_free_neighbor and has_unknown_neighbor:
                return True

        # If not both conditions are met, return False   
        return False

    
    
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

   

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.is_navigating = False
       
        


def main(args=None):
    rclpy.init(args=args)

    node = TurtleBot4_Frontier_Explorer()

    while node.map_data is None:
        rclpy.spin_once(node)
    while rclpy.ok():
        if not node.is_navigating:
            node.send_goal()
            node.is_navigating = True
        rclpy.spin_once(node)
   


if __name__ == '__main__':
    main()