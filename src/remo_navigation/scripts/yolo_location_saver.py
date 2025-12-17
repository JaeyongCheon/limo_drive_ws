#!/usr/bin/env python3
"""
YOLO 탐지 시 로봇 위치 저장 노드
YOLO에서 특정 객체를 탐지하면 현재 로봇의 odom 위치를 저장합니다.
"""

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import json
from datetime import datetime


class YoloLocationSaver(Node):
    def __init__(self):
        super().__init__('yolo_location_saver')
        
        # Parameters
        self.declare_parameter('target_classes', ['person', 'car', 'bottle'])  # 저장할 객체 클래스
        self.declare_parameter('detection_confidence', 0.5)  # 최소 신뢰도
        self.declare_parameter('save_file', '/home/wego/yolo_detections.json')
        
        self.target_classes = self.get_parameter('target_classes').value
        self.min_confidence = self.get_parameter('detection_confidence').value
        self.save_file = self.get_parameter('save_file').value
        
        # 현재 로봇 위치 저장
        self.current_odom = None
        self.detection_locations = []
        
        # Subscribers
        self.yolo_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',  # YOLO 탐지 결과 토픽
            self.yolo_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher - Nav2 목표 지점으로 사용 가능
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            '/yolo_waypoint',
            10
        )
        
        self.get_logger().info(f'YOLO Location Saver initialized')
        self.get_logger().info(f'Target classes: {self.target_classes}')
        self.get_logger().info(f'Saving to: {self.save_file}')
    
    def odom_callback(self, msg: Odometry):
        """현재 로봇 위치 업데이트"""
        self.current_odom = msg
    
    def yolo_callback(self, msg: DetectionArray):
        """YOLO 탐지 결과 처리"""
        if self.current_odom is None:
            self.get_logger().warn('No odom data available yet')
            return
        
        for detection in msg.detections:
            # 클래스 이름과 신뢰도 확인
            class_name = detection.class_name
            confidence = detection.score
            
            if class_name in self.target_classes and confidence >= self.min_confidence:
                # 현재 로봇 위치 저장
                position = self.current_odom.pose.pose.position
                orientation = self.current_odom.pose.pose.orientation
                
                detection_data = {
                    'timestamp': datetime.now().isoformat(),
                    'class': class_name,
                    'confidence': float(confidence),
                    'robot_position': {
                        'x': float(position.x),
                        'y': float(position.y),
                        'z': float(position.z)
                    },
                    'robot_orientation': {
                        'x': float(orientation.x),
                        'y': float(orientation.y),
                        'z': float(orientation.z),
                        'w': float(orientation.w)
                    }
                }
                
                self.detection_locations.append(detection_data)
                self.save_to_file()
                
                # Nav2 waypoint로 퍼블리시
                self.publish_waypoint(position, orientation)
                
                self.get_logger().info(
                    f'Detected {class_name} (conf: {confidence:.2f}) at '
                    f'x={position.x:.2f}, y={position.y:.2f}'
                )
    
    def publish_waypoint(self, position, orientation):
        """탐지 위치를 Nav2 waypoint로 퍼블리시"""
        waypoint = PoseStamped()
        waypoint.header = Header()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'odom'
        waypoint.pose.position = position
        waypoint.pose.orientation = orientation
        
        self.waypoint_pub.publish(waypoint)
    
    def save_to_file(self):
        """탐지 위치를 JSON 파일로 저장"""
        try:
            with open(self.save_file, 'w') as f:
                json.dump(self.detection_locations, f, indent=2)
            self.get_logger().info(f'Saved {len(self.detection_locations)} detections')
        except Exception as e:
            self.get_logger().error(f'Failed to save file: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloLocationSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
