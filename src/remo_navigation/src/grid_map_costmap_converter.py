#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
import numpy as np
import math


class GridMapCostmapConverter(Node):
    """
    Converts Grid Map traversability layer to Nav2 costmap format
    """
    
    def __init__(self):
        super().__init__('grid_map_costmap_converter')
        
        # Declare parameters
        self.declare_parameter('grid_map_topic', '/grid_map')
        self.declare_parameter('costmap_topic', '/grid_map_costmap')
        self.declare_parameter('traversability_layer', 'traversability')
        self.declare_parameter('inflation_radius', 0.5)
        
        # Get parameters
        grid_map_topic = self.get_parameter('grid_map_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value
        self.traversability_layer = self.get_parameter('traversability_layer').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        
        # Subscriber
        self.grid_map_sub = self.create_subscription(
            GridMap,
            grid_map_topic,
            self.grid_map_callback,
            10
        )
        
        # Publisher
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            costmap_topic,
            10
        )
        
        self.get_logger().info('Grid Map to Costmap converter started')
    
    def grid_map_callback(self, msg):
        """Convert Grid Map to OccupancyGrid costmap"""
        try:
            # Find traversability layer index
            if self.traversability_layer not in msg.layers:
                self.get_logger().warn(
                    f'Traversability layer "{self.traversability_layer}" not found in Grid Map',
                    throttle_duration_sec=1.0
                )
                return
            
            layer_idx = msg.layers.index(self.traversability_layer)
            
            # Extract layer data
            layer_data = msg.data[layer_idx].data
            
            # Convert to numpy array
            # Grid Map data is stored row-major
            # Need to determine dimensions from Grid Map info
            resolution = msg.info.resolution
            length_x = msg.info.length_x
            length_y = msg.info.length_y
            
            width = int(length_x / resolution)
            height = int(length_y / resolution)
            
            # Reshape data
            traversability_array = np.array(layer_data).reshape((height, width))
            
            # Convert traversability (0.0-1.0) to costmap (0-100)
            # 0.0 traversability = 100 (lethal obstacle)
            # 1.0 traversability = 0 (free space)
            costmap_array = (1.0 - traversability_array) * 100.0
            
            # Handle NaN values (set to unknown: -1)
            costmap_array = np.nan_to_num(costmap_array, nan=-1.0)
            
            # Clip to valid range [0, 100] or -1 for unknown
            costmap_array = np.clip(costmap_array, -1.0, 100.0)
            
            # Apply inflation (simplified - can be improved)
            costmap_array = self.apply_inflation(costmap_array, resolution)
            
            # Create OccupancyGrid message
            costmap_msg = OccupancyGrid()
            costmap_msg.header = msg.header
            costmap_msg.info.resolution = resolution
            costmap_msg.info.width = width
            costmap_msg.info.height = height
            costmap_msg.info.origin.position.x = msg.info.pose.position.x - length_x / 2.0
            costmap_msg.info.origin.position.y = msg.info.pose.position.y - length_y / 2.0
            costmap_msg.info.origin.position.z = 0.0
            costmap_msg.info.origin.orientation = msg.info.pose.orientation
            
            # Flatten and convert to int8
            costmap_msg.data = costmap_array.astype(np.int8).flatten().tolist()
            
            # Publish
            self.costmap_pub.publish(costmap_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error converting Grid Map to costmap: {e}')
    
    def apply_inflation(self, costmap_array, resolution):
        """Apply simple inflation to costmap"""
        # This is a simplified inflation - for production use Nav2's inflation layer
        inflated = costmap_array.copy()
        kernel_size = int(self.inflation_radius / resolution)
        
        if kernel_size < 1:
            return inflated
        
        height, width = costmap_array.shape
        
        for i in range(height):
            for j in range(width):
                if costmap_array[i, j] > 50:  # Obstacle
                    # Inflate around obstacles
                    for di in range(-kernel_size, kernel_size + 1):
                        for dj in range(-kernel_size, kernel_size + 1):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < height and 0 <= nj < width:
                                distance = math.sqrt(di*di + dj*dj) * resolution
                                if distance <= self.inflation_radius:
                                    # Reduce cost with distance
                                    cost = costmap_array[i, j] * (1.0 - distance / self.inflation_radius)
                                    if inflated[ni, nj] < cost:
                                        inflated[ni, nj] = cost
        
        return inflated


def main(args=None):
    rclpy.init(args=args)
    node = GridMapCostmapConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

