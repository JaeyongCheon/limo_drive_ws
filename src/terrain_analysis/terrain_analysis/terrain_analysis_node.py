#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from geometry_msgs.msg import Point, Quaternion, TransformStamped
import numpy as np
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import StaticTransformBroadcaster
def euler_from_quaternion(quaternion):
    """Convert quaternion to euler angles (roll, pitch, yaw)"""
    x, y, z, w = quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quaternion_matrix(quaternion):
    """Convert quaternion to 4x4 rotation matrix"""
    x, y, z, w = quaternion
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w), 0],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w), 0],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y), 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)


class TerrainAnalysisNode(Node):
    def __init__(self):
        super().__init__('terrain_analysis_node')
        
        # Declare parameters
        self.declare_parameter('map_length_x', 10.0)
        self.declare_parameter('map_length_y', 10.0)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_slope', 0.5)
        self.declare_parameter('max_roughness', 0.2)
        
        # Get parameters
        self.map_length_x = self.get_parameter('map_length_x').value
        self.map_length_y = self.get_parameter('map_length_y').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_slope = self.get_parameter('max_slope').value
        self.max_roughness = self.get_parameter('max_roughness').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize grid map data structure
        self.map_size_x = int(self.map_length_x / self.map_resolution)
        self.map_size_y = int(self.map_length_y / self.map_resolution)
        
        # Initialize layers
        self.layers = ['elevation', 'slope', 'roughness', 'traversability']
        self.grid_data = {}
        for layer in self.layers:
            self.grid_data[layer] = np.full(
                (self.map_size_x, self.map_size_y), 
                float('nan'), 
                dtype=np.float32
            )
        
        # Robot state
        self.robot_position = np.array([0.0, 0.0])
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.initialized = False
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create subscribers
        scan_topic = self.get_parameter('scan_topic').value
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        imu_topic = self.get_parameter('imu_topic').value
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
        
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        camera_topic = self.get_parameter('camera_topic').value
        if camera_topic:
            self.camera_sub = self.create_subscription(
                Image,
                camera_topic,
                self.camera_callback,
                10
            )
        
        # Create publisher
        self.grid_map_pub = self.create_publisher(
            GridMap,
            'grid_map',
            10
        )
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_grid_map)
        
        self.get_logger().info(f'Terrain analysis node started')
        self.get_logger().info(f'Grid map size: {self.map_size_x} x {self.map_size_y} cells')
        self.get_logger().info(f'Resolution: {self.map_resolution} m/cell')
    
    def scan_callback(self, msg):
        try:
            # Get transform from laser frame to map frame
            # Use latest available transform instead of exact timestamp to avoid extrapolation
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),  # Use latest available
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Convert laser scan to elevation data
            self.update_elevation_from_scan(msg, transform)
            self.initialized = True
            
        except TransformException as ex:
            # TF 버퍼에 데이터가 충분하지 않을 수 있음 - 무시하고 계속 진행
            self.get_logger().debug(
                f'Could not transform laser scan: {ex}',
                throttle_duration_sec=5.0
            )
            return
    
    def imu_callback(self, msg):
        # Extract roll and pitch from quaternion
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        # Update slope based on IMU orientation
        self.current_roll = roll
        self.current_pitch = pitch
        self.update_slope()
    
    def odom_callback(self, msg):
        # Update robot position for grid map centering
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y
    
    def camera_callback(self, msg):
        # TODO: Implement terrain classification from camera
        # This could use computer vision to detect mud, vegetation, etc.
        pass
    
    def update_elevation_from_scan(self, scan, transform):
        """Convert laser scan points to grid map elevation"""
        # Get transform matrix
        t = transform.transform.translation
        q = transform.transform.rotation
        quaternion = [q.x, q.y, q.z, q.w]
        rotation_matrix = quaternion_matrix(quaternion)
        translation = np.array([t.x, t.y, t.z, 0.0])  # 4D로 확장
        
        # Process each scan point
        for i, range_val in enumerate(scan.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            
            angle = scan.angle_min + i * scan.angle_increment
            
            # Point in laser frame
            x_laser = range_val * math.cos(angle)
            y_laser = range_val * math.sin(angle)
            z_laser = 0.0
            point_laser = np.array([x_laser, y_laser, z_laser, 1.0])
            
            # Transform to map frame
            point_map_homogeneous = rotation_matrix @ point_laser + translation
            point_map = point_map_homogeneous[:3]  # 처음 3개 요소만 사용 (x, y, z)
            
            # Update grid map elevation
            map_x = int((point_map[0] - self.robot_position[0] + self.map_length_x / 2) / self.map_resolution)
            map_y = int((point_map[1] - self.robot_position[1] + self.map_length_y / 2) / self.map_resolution)
            
            if 0 <= map_x < self.map_size_x and 0 <= map_y < self.map_size_y:
                # Use minimum elevation (closest to ground)
                current_elevation = self.grid_data['elevation'][map_x, map_y]
                if math.isnan(current_elevation) or point_map[2] < current_elevation:
                    self.grid_data['elevation'][map_x, map_y] = point_map[2]
    
    def update_slope(self):
        """Calculate slope magnitude from roll and pitch"""
        slope_magnitude = math.sqrt(
            self.current_roll * self.current_roll + 
            self.current_pitch * self.current_pitch
        )
        
        # Update slope layer (simplified: using current IMU reading)
        # In a more sophisticated implementation, calculate slope per cell
        self.grid_data['slope'][:] = slope_magnitude
    
    def calculate_roughness(self):
        """Calculate roughness as variance of elevation in neighborhood"""
        roughness = np.full_like(self.grid_data['elevation'], float('nan'))
        
        # Kernel size for neighborhood
        kernel_size = 3
        
        for i in range(kernel_size // 2, self.map_size_x - kernel_size // 2):
            for j in range(kernel_size // 2, self.map_size_y - kernel_size // 2):
                # Get neighborhood elevations
                neighborhood = self.grid_data['elevation'][
                    i - kernel_size // 2:i + kernel_size // 2 + 1,
                    j - kernel_size // 2:j + kernel_size // 2 + 1
                ]
                
                # Filter out NaN values
                valid_elevations = neighborhood[~np.isnan(neighborhood)]
                
                if len(valid_elevations) > 3:
                    variance = np.var(valid_elevations)
                    roughness[i, j] = math.sqrt(variance)
        
        self.grid_data['roughness'] = roughness
    
    def calculate_traversability(self):
        """Calculate traversability based on slope and roughness"""
        traversability = np.ones_like(self.grid_data['elevation'])
        
        # Get valid indices
        valid_mask = ~np.isnan(self.grid_data['slope']) & ~np.isnan(self.grid_data['roughness'])
        
        # Reduce traversability based on slope
        slope = self.grid_data['slope']
        slope_mask = slope > self.max_slope
        traversability[slope_mask] = 0.0
        
        valid_slope_mask = valid_mask & (slope <= self.max_slope)
        traversability[valid_slope_mask] *= (1.0 - slope[valid_slope_mask] / self.max_slope)
        
        # Reduce traversability based on roughness
        roughness = self.grid_data['roughness']
        roughness_mask = roughness > self.max_roughness
        traversability[roughness_mask] = 0.0
        
        valid_roughness_mask = valid_mask & (roughness <= self.max_roughness)
        traversability[valid_roughness_mask] *= (1.0 - roughness[valid_roughness_mask] / self.max_roughness)
        
        # Clamp to [0, 1]
        traversability = np.clip(traversability, 0.0, 1.0)
        self.grid_data['traversability'] = traversability
    
    def publish_grid_map(self):
        if not self.initialized:
            return
        
        # Update derived layers
        self.calculate_roughness()
        self.calculate_traversability()
        
        # Create GridMap message
        grid_map_msg = GridMap()
        grid_map_msg.header.stamp = self.get_clock().now().to_msg()
        grid_map_msg.header.frame_id = self.map_frame
        
        # Set basic info
        grid_map_msg.info.resolution = self.map_resolution
        grid_map_msg.info.length_x = self.map_length_x
        grid_map_msg.info.length_y = self.map_length_y
        
        # Set pose (center of map at robot position)
        grid_map_msg.info.pose.position.x = self.robot_position[0]
        grid_map_msg.info.pose.position.y = self.robot_position[1]
        grid_map_msg.info.pose.position.z = 0.0
        grid_map_msg.info.pose.orientation.w = 1.0
        
        # Add layers
        grid_map_msg.layers = self.layers
        
        # Convert numpy arrays to Float32MultiArray
        for layer in self.layers:
            layer_data = Float32MultiArray()
            
            # Set layout dimensions for proper RViz visualization
            dim_x = MultiArrayDimension()
            dim_x.label = "column_index"
            dim_x.size = self.map_size_x
            dim_x.stride = self.map_size_x * self.map_size_y
            
            dim_y = MultiArrayDimension()
            dim_y.label = "row_index"
            dim_y.size = self.map_size_y
            dim_y.stride = self.map_size_y
            
            layer_data.layout.dim = [dim_x, dim_y]
            layer_data.layout.data_offset = 0
            
            # Flatten and convert to list
            flat_data = self.grid_data[layer].flatten()
            layer_data.data = flat_data.tolist()
            grid_map_msg.data.append(layer_data)
        
        # Publish
        self.grid_map_pub.publish(grid_map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

