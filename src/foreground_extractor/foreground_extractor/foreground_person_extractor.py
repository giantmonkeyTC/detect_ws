#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import open3d as o3d

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        # 设置Qos策略
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=0
        )
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/foreground_points',
            self.cloud_callback,
            qos_profile=qos_profile
        )
        # 发布边界框（RViz 可视化）
        self.marker_pub = self.create_publisher(MarkerArray, '/human_cluster', qos_profile=qos_profile)
        
        # 参数配置
        self.voxel_size = 0.05   # 降采样体素大小 (m)
        self.eps = 0.13          # DBSCAN 聚类半径 (m)  0.2  0.15
        self.min_samples = 5     # 最小聚类点数  10   5

    def preprocess_point_cloud(self, points):
        """点云预处理（降采样 + 地面去除）"""
        pcd = o3d.geometry.PointCloud()

        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 降采样
        pcd = pcd.voxel_down_sample(self.voxel_size)
        
        # 地面去除（RANSAC）
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.1,
            ransac_n=3,
            num_iterations=100
        )
        pcd = pcd.select_by_index(inliers, invert=True)   #保留非地面点
        
        return np.asarray(pcd.points)

    def cluster_points(self, points):
        """DBSCAN 聚类"""  
        # 通过DBSCAN算法进行聚类
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = clustering.labels_
        
        # 过滤噪声点（label=-1）
        valid_points = points[labels != -1]
        valid_labels = labels[labels != -1]
        
        return valid_points, valid_labels

    def detect_humans(self, points, labels):
        """人形边界框检测"""
        human_boxes = []                     # !!! 其实可以为矩形，不为长方体
        unique_labels = np.unique(labels)    # 提取所有唯一的聚类标签
        
        for label in unique_labels:
            cluster = points[labels == label]
            min_bound = np.min(cluster, axis=0)    # 计算簇的最小xyz坐标    ---》
            max_bound = np.max(cluster, axis=0)    # 计算簇的最大xyz坐标    ---》计算得到其轴对齐边界框(AABB)
            
            # 人形尺寸过滤（单位：米）
            height = max_bound[2] - min_bound[2]  # z轴
            width = max_bound[1] - min_bound[1]   # y轴
            depth = max_bound[0] - min_bound[0]   # x轴s
            
            if (1.3 < height < 1.9) and (0.32 < width < 0.8) and (0.32 < depth < 0.8):
                human_boxes.append((min_bound, max_bound))
        
        return human_boxes

    def create_marker_array(self, boxes):
        """生成 RViz 边界框 MarkerArray"""
        marker_array = MarkerArray()
        
        for i, (min_bound, max_bound) in enumerate(boxes):
            marker = Marker()
            marker.header.frame_id = "map"  # 替换为你的点云坐标系
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = i
            
            # 计算中心点和尺寸

            center = (min_bound + max_bound) / 2
            scale = max_bound - min_bound
            
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]
            marker.scale.x = scale[0]
            marker.scale.y = scale[1]
            marker.scale.z = scale[2]
            
            marker.color.r = 1.0
            marker.color.a = 0.3  # 透明度
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            
            marker_array.markers.append(marker)
        
        return marker_array

    def cloud_callback(self, msg):
        """点云回调函数"""
        # 1. 从 PointCloud2 提取数据
        field_names = ("x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range")
        #pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_data = pc2.read_points(msg, skip_nans=True, field_names=field_names)

        points = np.array(list(pc_data))

        xyz = np.column_stack([points['x'], points['y'], points['z']])  # (N,3)坐标
        
        # 2. 预处理   过滤地面  
        filtered_points = self.preprocess_point_cloud(xyz)
        if len(filtered_points) == 0:
            return
        
        # 3. 聚类
        clustered_points, labels = self.cluster_points(filtered_points)
        if len(clustered_points) == 0:   # 没有"有效簇"的点
            return
        
        # 4. 人形检测
        human_boxes = self.detect_humans(clustered_points, labels)
        
        # 5. 发布边界框到 RViz
        marker_array = self.create_marker_array(human_boxes)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()