#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sensor_msgs_py.point_cloud2 as pc2


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


class TerrainAnalysisNodeLimo(Node):
    def __init__(self):
        super().__init__('terrain_analysis_node_limo')
        
        # Declare parameters (LIMO Pro 최적화 기본값)
        self.declare_parameter('map_length_x', 8.0)
        self.declare_parameter('map_length_y', 8.0)
        self.declare_parameter('map_resolution', 0.05)  # LIMO Pro: 5cm 해상도
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('depth_topic', '/camera/depth/points')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('depth_processing_rate', 5.0)
        self.declare_parameter('max_slope', 0.5)
        self.declare_parameter('max_roughness', 0.2)
        self.declare_parameter('use_depth_camera', True)
        self.declare_parameter('depth_max_range', 5.0)  # 깊이 카메라 최대 거리
        self.declare_parameter('depth_min_range', 0.3)  # 깊이 카메라 최소 거리
        self.declare_parameter('scan_max_range', 4.0)   # LiDAR 최대 거리
        
        # Get parameters
        self.map_length_x = self.get_parameter('map_length_x').value
        self.map_length_y = self.get_parameter('map_length_y').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_slope = self.get_parameter('max_slope').value
        self.max_roughness = self.get_parameter('max_roughness').value
        self.use_depth_camera = self.get_parameter('use_depth_camera').value
        self.depth_max_range = self.get_parameter('depth_max_range').value
        self.depth_min_range = self.get_parameter('depth_min_range').value
        self.scan_max_range = self.get_parameter('scan_max_range').value
        publish_rate = self.get_parameter('publish_rate').value
        depth_processing_rate = self.get_parameter('depth_processing_rate').value
        
        # Initialize grid map data structure
        self.map_size_x = int(self.map_length_x / self.map_resolution)
        self.map_size_y = int(self.map_length_y / self.map_resolution)
        
        # Initialize layers
        self.layers = ['elevation', 'slope', 'roughness', 'traversability']
        self.grid_data = {}
        self.grid_count = {}  # 각 셀의 측정 횟수 (평균 계산용)
        
        for layer in self.layers:
            self.grid_data[layer] = np.full(
                (self.map_size_x, self.map_size_y), 
                float('nan'), 
                dtype=np.float32
            )
        
        self.grid_count = np.zeros((self.map_size_x, self.map_size_y), dtype=np.int32)
        
        # Robot state
        self.robot_position = np.array([0.0, 0.0])
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.initialized = False
        
        # 실시간 맵 모드 (매 프레임 초기화 - 뭉개짐 방지)
        self.use_realtime_mode = True
        
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
        
        # Depth camera subscriber (Orbbec Dabai)
        if self.use_depth_camera:
            depth_topic = self.get_parameter('depth_topic').value
            self.depth_sub = self.create_subscription(
                PointCloud2,
                depth_topic,
                self.depth_callback,
                5  # 낮은 큐 크기 (대용량 데이터)
            )
            self.get_logger().info(f'Depth camera enabled: {depth_topic}')
            
            # Depth processing timer (더 느린 주기)
            self.depth_processing_enabled = True
            self.depth_timer = self.create_timer(
                1.0 / depth_processing_rate, 
                self.toggle_depth_processing
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
        
        self.get_logger().info(f'Terrain analysis node (LIMO Pro) started')
        self.get_logger().info(f'Grid map size: {self.map_size_x} x {self.map_size_y} cells')
        self.get_logger().info(f'Resolution: {self.map_resolution} m/cell')
    
    def toggle_depth_processing(self):
        """깊이 카메라 처리 속도 조절을 위한 토글"""
        pass  # 실제로는 depth_callback에서 처리 여부를 판단
    
    def scan_callback(self, msg):
        """2D LiDAR 스캔 처리 - 로봇 중심 좌표로 직접 처리"""
        # 실시간 모드: 매 스캔마다 맵 초기화
        if self.use_realtime_mode:
            self.reset_map()
        
        # LiDAR 데이터를 직접 로봇 중심 좌표로 처리 (TF 변환 불필요)
        self.update_elevation_from_scan_direct(msg)
        self.initialized = True
    
    def depth_callback(self, msg):
        """깊이 카메라 포인트 클라우드 처리 - 로봇 중심 좌표로 직접 처리"""
        if not self.use_depth_camera:
            return
        
        try:
            self.update_elevation_from_depth_direct(msg)
            self.initialized = True
        except Exception as ex:
            self.get_logger().warn(
                f'Error processing depth data: {ex}',
                throttle_duration_sec=5.0
            )
    
    def imu_callback(self, msg):
        """IMU 데이터로 로봇 자세 업데이트"""
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        self.current_roll = roll
        self.current_pitch = pitch
        self.update_slope()
    
    def odom_callback(self, msg):
        """오도메트리로 로봇 위치 업데이트"""
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y
    
    def update_elevation_from_scan_direct(self, scan):
        """2D LiDAR 스캔을 로봇 중심 좌표로 직접 처리"""
        for i, range_val in enumerate(scan.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            # 범위 필터링 (scan_max_range로 제한)
            if range_val < scan.range_min or range_val > min(scan.range_max, self.scan_max_range):
                continue
            
            angle = scan.angle_min + i * scan.angle_increment
            
            # 로봇 중심 좌표로 직접 계산 (LiDAR는 거의 로봇 중심에 있음)
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            z = 0.0  # 2D LiDAR는 높이 정보 없음
            
            # Update grid map (로봇 중심 좌표)
            map_x = int((x + self.map_length_x / 2) / self.map_resolution)
            map_y = int((y + self.map_length_y / 2) / self.map_resolution)
            
            if 0 <= map_x < self.map_size_x and 0 <= map_y < self.map_size_y:
                current_elevation = self.grid_data['elevation'][map_x, map_y]
                if math.isnan(current_elevation):
                    self.grid_data['elevation'][map_x, map_y] = z
                else:
                    self.grid_data['elevation'][map_x, map_y] = min(current_elevation, z)
    
    def update_elevation_from_depth_direct(self, cloud_msg):
        """깊이 카메라 포인트 클라우드를 로봇 중심 좌표로 직접 처리"""
        # 포인트 클라우드 읽기 (다운샘플링: LIMO Pro 성능 최적화)
        point_count = 0
        skip_points = 15  # 15개 중 1개만 처리 (성능 향상)
        
        for i, point in enumerate(pc2.read_points(cloud_msg, skip_nans=True, 
                                                   field_names=("x", "y", "z"))):
            if i % skip_points != 0:
                continue
                
            # 카메라 좌표계: x=전방, y=좌측, z=위
            cam_x, cam_y, cam_z = point
            
            # 거리 필터링
            distance = math.sqrt(cam_x*cam_x + cam_y*cam_y + cam_z*cam_z)
            if distance < self.depth_min_range or distance > self.depth_max_range:
                continue
            
            # 카메라 → 로봇 좌표 변환 (카메라가 로봇 전방에 위치)
            # 간단히 x, y를 그대로 사용 (카메라와 로봇 좌표계가 비슷하다고 가정)
            robot_x = cam_x
            robot_y = cam_y
            robot_z = cam_z
            
            # Update grid map (로봇 중심 좌표)
            map_x = int((robot_x + self.map_length_x / 2) / self.map_resolution)
            map_y = int((robot_y + self.map_length_y / 2) / self.map_resolution)
            
            if 0 <= map_x < self.map_size_x and 0 <= map_y < self.map_size_y:
                current_elevation = self.grid_data['elevation'][map_x, map_y]
                
                if math.isnan(current_elevation):
                    self.grid_data['elevation'][map_x, map_y] = robot_z
                else:
                    # 같은 셀에 여러 포인트가 있으면 최소값 사용 (바닥 감지)
                    self.grid_data['elevation'][map_x, map_y] = min(
                        current_elevation, robot_z
                    )
            
            point_count += 1
            
            # LIMO Pro 성능에 맞게 최대 포인트 수 제한
            if point_count > 5000:
                break
    
    def update_slope(self):
        """IMU 기반 경사 계산 및 로컬 경사도 계산"""
        # 전역 경사 (IMU 기반)
        global_slope = math.sqrt(
            self.current_roll * self.current_roll + 
            self.current_pitch * self.current_pitch
        )
        
        # 로컬 경사 (고도 변화 기반)
        slope = np.full_like(self.grid_data['elevation'], float('nan'))
        
        for i in range(1, self.map_size_x - 1):
            for j in range(1, self.map_size_y - 1):
                center_elevation = self.grid_data['elevation'][i, j]
                
                if math.isnan(center_elevation):
                    continue
                
                # 이웃 셀과의 고도 차이 계산
                neighbors = [
                    self.grid_data['elevation'][i-1, j],
                    self.grid_data['elevation'][i+1, j],
                    self.grid_data['elevation'][i, j-1],
                    self.grid_data['elevation'][i, j+1]
                ]
                
                valid_neighbors = [n for n in neighbors if not math.isnan(n)]
                
                if len(valid_neighbors) > 0:
                    max_diff = max([abs(center_elevation - n) for n in valid_neighbors])
                    slope[i, j] = math.atan(max_diff / self.map_resolution)
        
        self.grid_data['slope'] = slope
    
    def calculate_roughness(self):
        """고도 변화의 표준편차로 거칠기 계산"""
        roughness = np.full_like(self.grid_data['elevation'], float('nan'))
        
        kernel_size = 3  # LIMO Pro: 해상도에 맞게 조정
        
        for i in range(kernel_size // 2, self.map_size_x - kernel_size // 2):
            for j in range(kernel_size // 2, self.map_size_y - kernel_size // 2):
                neighborhood = self.grid_data['elevation'][
                    i - kernel_size // 2:i + kernel_size // 2 + 1,
                    j - kernel_size // 2:j + kernel_size // 2 + 1
                ]
                
                valid_elevations = neighborhood[~np.isnan(neighborhood)]
                
                if len(valid_elevations) > 5:
                    variance = np.var(valid_elevations)
                    roughness[i, j] = math.sqrt(variance)
        
        self.grid_data['roughness'] = roughness
    
    def calculate_traversability(self):
        """경사와 거칠기 기반 통과 가능성 계산"""
        traversability = np.ones_like(self.grid_data['elevation'])
        
        valid_mask = ~np.isnan(self.grid_data['slope']) & ~np.isnan(self.grid_data['roughness'])
        
        # 경사 기반 감소
        slope = self.grid_data['slope']
        slope_mask = slope > self.max_slope
        traversability[slope_mask] = 0.0
        
        valid_slope_mask = valid_mask & (slope <= self.max_slope)
        traversability[valid_slope_mask] *= (1.0 - slope[valid_slope_mask] / self.max_slope)
        
        # 거칠기 기반 감소
        roughness = self.grid_data['roughness']
        roughness_mask = roughness > self.max_roughness
        traversability[roughness_mask] = 0.0
        
        valid_roughness_mask = valid_mask & (roughness <= self.max_roughness)
        traversability[valid_roughness_mask] *= (1.0 - roughness[valid_roughness_mask] / self.max_roughness)
        
        traversability = np.clip(traversability, 0.0, 1.0)
        self.grid_data['traversability'] = traversability
    
    def reset_map(self):
        """맵 데이터 초기화"""
        for layer in self.layers:
            self.grid_data[layer] = np.full(
                (self.map_size_x, self.map_size_y), 
                float('nan'), 
                dtype=np.float32
            )
        self.grid_count = np.zeros((self.map_size_x, self.map_size_y), dtype=np.int32)
    
    def publish_grid_map(self):
        """Grid Map 발행"""
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
        
        # Set pose (base_link 프레임 중심 - 로봇이 항상 맵 중앙)
        grid_map_msg.info.pose.position.x = 0.0
        grid_map_msg.info.pose.position.y = 0.0
        grid_map_msg.info.pose.position.z = 0.0
        grid_map_msg.info.pose.orientation.w = 1.0
        
        # Add layers
        grid_map_msg.layers = self.layers
        
        # Convert numpy arrays to Float32MultiArray
        for layer in self.layers:
            layer_data = Float32MultiArray()
            
            # Set layout dimensions
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
            
            flat_data = self.grid_data[layer].flatten()
            layer_data.data = flat_data.tolist()
            grid_map_msg.data.append(layer_data)
        
        # Publish
        self.grid_map_pub.publish(grid_map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalysisNodeLimo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

