import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import open3d as o3d
import numpy as np

##使用pcd文件作为背景
class ForegroundExtractor(Node):
    def __init__(self):
        super().__init__('foreground_extractor')
        
        # 声明参数
        self.declare_parameter('background_path', '/home/lance/detect_ws/src/foreground_extractor/background.pcd')
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('height_threshold', 1.4)

        # 加载参数
        bg_path = self.get_parameter('background_path').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.height_threshold = self.get_parameter('height_threshold').value

        # 加载背景点云并构建体素集合
        self.background_voxels = self._load_background_voxels(bg_path)
        self.get_logger().info(f'Loaded {len(self.background_voxels)} background voxels')

        # 配置QoS策略
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=0
        )

        # 创建订阅和发布
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.cloud_callback,
            qos_profile=qos_profile
        )
        self.publisher = self.create_publisher(PointCloud2, 'foreground_points', 10)

        

    def _load_background_voxels(self, path):
        """加载背景点云并构建体素集合"""
        pcd = o3d.io.read_point_cloud(path)
        if not pcd.has_points():
            raise ValueError("Empty background point cloud")

        downsampled = pcd.voxel_down_sample(self.voxel_size)
        points = np.asarray(downsampled.points)
        
        voxel_dict = {}
        for pt in points:
            x_idx = int(np.floor(pt[0] / self.voxel_size))
            y_idx = int(np.floor(pt[1] / self.voxel_size))
            z_idx = int(np.floor(pt[2] / self.voxel_size))
            voxel_dict[(x_idx, y_idx, z_idx)] = pt[2]  # 保存体素平均高度
        return voxel_dict

    def cloud_callback(self, msg):
        """点云处理回调函数"""
        # 读取点云数据
        points = list(pc2.read_points(msg, skip_nans=True))
        if not points:
            return
        
        tf = np.array([
            [-0.643753886223, -0.209196105599, 0.736082792282, -4.610776328481],
            [0.165232002735, -0.977215886116, -0.133220076561, -2.006507927920],
            [0.747180938721, 0.035863488913, 0.663652420044, 2.022653023053],
            [0.0, 0.0, 0.0, 1.0]
        ])
        # 处理每个点
        foreground = []
        for p in points:
            x, y, z = p[0], p[1], p[2]

            point_homogeneous = np.array([x, y, z, 1.0])
            transformed_point = np.dot(tf, point_homogeneous)
            x_tf, y_tf, z_tf = transformed_point[:3]  # 取前三个分量

            key = (
                int(np.floor(x_tf / self.voxel_size)),
                int(np.floor(y_tf / self.voxel_size)),
                int(np.floor(z_tf / self.voxel_size))
            )

            # 判断是否前景点
            if key not in self.background_voxels:
                foreground.append(p)
            elif z_tf > (self.background_voxels[key] + self.height_threshold):
                foreground.append(p)

        # 发布前景点云
        if foreground:
            header = msg.header
            header.stamp = self.get_clock().now().to_msg()
            cloud = pc2.create_cloud(header, msg.fields, foreground)
            self.publisher.publish(cloud)
            self.get_logger().debug(f'Published {len(foreground)} foreground points')

def main(args=None):
    rclpy.init(args=args)
    node = ForegroundExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()