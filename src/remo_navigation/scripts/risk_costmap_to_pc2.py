#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class RiskCostmapToPointCloud(Node):
    """
    /grid_map_costmap(OccupancyGrid)에서 lethal_threshold 이상 셀만 뽑아
    PointCloud2로 퍼블리시 -> Nav2 obstacle/voxel layer에 observation source로 넣기
    """

    def __init__(self):
        super().__init__('risk_costmap_to_pointcloud')

        self.declare_parameter('in_topic', '/grid_map_costmap')
        self.declare_parameter('out_topic', '/risk_obstacles')
        self.declare_parameter('lethal_threshold', 70)     # occ >= 이 값이면 장애물
        self.declare_parameter('z', 0.0)                   # 점의 z (2D면 0.0)
        self.declare_parameter('step', 1)                  # 1이면 모든 셀, 2면 2칸마다 샘플링 (성능용)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.lethal_threshold = int(self.get_parameter('lethal_threshold').value)
        self.z = float(self.get_parameter('z').value)
        self.step = max(1, int(self.get_parameter('step').value))

        self.sub = self.create_subscription(OccupancyGrid, self.in_topic, self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, self.out_topic, 10)

        self.get_logger().info(
            f"risk_costmap_to_pointcloud: {self.in_topic} -> {self.out_topic} "
            f"(lethal_threshold={self.lethal_threshold}, step={self.step})"
        )

    def cb(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16)  # -1~100
        if data.size != w * h or w == 0 or h == 0:
            return

        # (h, w)로 reshape (row-major)
        grid = data.reshape((h, w))

        pts = []
        th = self.lethal_threshold

        # lethal 셀만 포인트로 변환
        for iy in range(0, h, self.step):
            row = grid[iy]
            for ix in range(0, w, self.step):
                occ = int(row[ix])
                if occ >= th:
                    # 셀 중심 좌표
                    x = ox + (ix + 0.5) * res
                    y = oy + (iy + 0.5) * res
                    pts.append((float(x), float(y), float(self.z)))

        pc = point_cloud2.create_cloud_xyz32(msg.header, pts)
        self.pub.publish(pc)

def main():
    rclpy.init()
    node = RiskCostmapToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
