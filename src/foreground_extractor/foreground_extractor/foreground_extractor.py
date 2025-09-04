import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import open3d as o3d
import numpy as np

#使用pcd文件作为背景
class ForegroundExtractor(Node):
    def __init__(self):
        super().__init__('foreground_extractor')
        
        # 声明参数
        self.declare_parameter('background_path', '/home/lance/detect_ws/src/foreground_extractor/background.pcd')
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('height_threshold', 0.4)

        # 加载参数
        bg_path = self.get_parameter('background_path').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.height_threshold = self.get_parameter('height_threshold').value

        # 加载背景点云并构建体素集合
        # self.background_voxels = self._load_background_voxels(bg_path)
        # self.get_logger().info(f'Loaded {len(self.background_voxels)} background voxels')

        # 配置QoS策略   sub---> best_effort
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=0
        )

        # 定义坐标变换矩阵
        self.tf = np.array([
            [-0.643753886223, -0.209196105599, 0.736082792282, -4.610776328481],
            [0.165232002735, -0.977215886116, -0.133220076561, -2.006507927920],
            [0.747180938721, 0.035863488913, 0.663652420044, 2.022653023053],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # 初始化背景点云标志
        self.background_initialized = False
        self.background_voxels = None

        # 创建订阅和发布
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.cloud_callback,
            qos_profile=qos_profile
        )
        self.publisher = self.create_publisher(PointCloud2, 'foreground_points', 10)

    def _transform_point(self, point):
        """应用坐标变换到单个点"""
        point_homogeneous = np.array([point[0], point[1], point[2], 1.0])
        transformed_point = np.dot(self.tf, point_homogeneous)
        return transformed_point[:3]  # 返回前三个分量(x,y,z)
    

    def _initialize_background(self, points):

        # ！！！针对背景点云进行处理（同时通过函数_transform_point来进行坐标转化）

        """使用第一帧点云初始化背景(应用坐标变换)"""
        self.get_logger().info("Initializing background with first frame...")
        
        # 应用坐标变换到所有点
        transformed_points = []
        for pt in points:
            x_tf, y_tf, z_tf = self._transform_point(pt)     #是否需要再转化一次？
            transformed_points.append([x_tf, y_tf, z_tf])
        
        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(transformed_points)
        
        # 下采样
        downsampled = pcd.voxel_down_sample(self.voxel_size)   # voxel_size: 0.2
        points = np.asarray(downsampled.points)
        
        # 构建体素字典(使用变换后的坐标)
        voxel_dict = {}
        for pt in points:
            x_idx = int(np.floor(pt[0] / self.voxel_size))   
            y_idx = int(np.floor(pt[1] / self.voxel_size))
            z_idx = int(np.floor(pt[2] / self.voxel_size))
            voxel_dict[(x_idx, y_idx, z_idx)] = pt[2]  # 保存体素平均高度  体素坐标--->高度
        
        self.background_voxels = voxel_dict  
        self.background_initialized = True   # indicate background pointcloud have initialized
        self.get_logger().info(f'Initialized background with {len(self.background_voxels)} voxels')


    def cloud_callback(self, msg):
        """点云处理回调函数"""
        z_threshold = 1.8  
        points = []
        # points = [p for p in pc2.read_points(msg, skip_nans=True) 
        #         if p[2] <= z_threshold]
         
        # 过滤掉高度大于2米的点云
        field_names = ("x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range")
        pc_data = pc2.read_points(msg, skip_nans=True, field_names=field_names)
        dtype = [
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('t', np.float32), ('reflectivity', np.float32),
            ('ring', np.uint16), ('ambient', np.float32), ('range', np.float32)
        ]
        points_array = np.array(list(pc_data), dtype=dtype)

        # 提取 x, y, z 并转换
        xyz = np.column_stack([points_array['x'], points_array['y'], points_array['z']])
        xyz_tf = np.apply_along_axis(self._transform_point, 1, xyz)   # 对每个点进行坐标转化

        # 过滤
        mask = xyz_tf[:, 2] < z_threshold
        filtered_points = points_array[mask]  # 直接过滤结构化数组    过滤转化后高度高于2m的全部点云数据

        # 更新变换后的坐标
        filtered_points['x'] = xyz_tf[mask, 0]      #查看一下原始点云的xyz，然后在这儿转化一次后的xyz，以及在_initialize_background是否又转化了一次+
        filtered_points['y'] = xyz_tf[mask, 1]
        filtered_points['z'] = xyz_tf[mask, 2]

        # 转换为列表（如果需要）
        points = filtered_points.tolist()

        # points是否为空
        if not points:
            return
        # 首先处理背景点云
        if not self.background_initialized:
            points_array = np.array([(p[0], p[1], p[2]) for p in points])
            self._initialize_background(points_array)
            return  # 跳过第一帧处理

        # 处理每个点
        foreground = []
        for p in points:
            x_tf, y_tf, z_tf = p[0], p[1], p[2]
            
            #x_tf, y_tf, z_tf = self._transform_point([x_tf,y_tf,z_tf])  # need to be removed or not
            key = (
                int(np.floor(x_tf / self.voxel_size)),    # voxel_size = 0.2  向下取整
                int(np.floor(y_tf / self.voxel_size)),
                int(np.floor(z_tf / self.voxel_size))
            )

            # 判断是否前景点   background_voxels为   若不在background中
            if key not in self.background_voxels:
                foreground.append(p)
            elif z_tf > (self.background_voxels[key] + self.height_threshold):
                foreground.append(p)

        # 发布前景点云
        if foreground:
            header = msg.header
            header.frame_id = "map"    # set foreground frame_id = map
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