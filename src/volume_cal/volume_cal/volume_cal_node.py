import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy



class VolumeCal(Node):
    def __init__(self):
        super().__init__('volume_cal_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic', '/ouster/points'),
                ('output_topic', '/processed_cloud'),
                ('x_min', -3.3), ('x_max', -1.0),
                ('y_min', -4.1), ('y_max', -1.9),
                ('z_min', -0.4), ('z_max', 0.4),
                ('transform_matrix', 
            [-0.643753886223, -0.209196105599, 0.736082792282, -4.610776328481,
            0.165232002735, -0.977215886116, -0.133220076561, -2.006507927920,
            0.747180938721, 0.035863488913, 0.663652420044, 2.022653023053,
            0.0, 0.0, 0.0, 1.0]), 
                ('output_frame', 'base_link')
            ])
        # 获取参数
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.crop_limits = {
            'x': (self.get_parameter('x_min').value, self.get_parameter('x_max').value),
            'y': (self.get_parameter('y_min').value, self.get_parameter('y_max').value),
            'z': (self.get_parameter('z_min').value, self.get_parameter('z_max').value)
        }
        self.tf_matrix = np.array(
            self.get_parameter('transform_matrix').value,
            dtype=np.float32).reshape(4, 4)
        self.output_frame = self.get_parameter('output_frame').value

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=0
        )

        # 初始化订阅和发布
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.process_cloud,
            qos_profile)
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10)
        
        self.get_logger().info("节点已启动，等待点云数据...")

    def process_cloud(self, msg):
        # 读取点云数据
        pc_gen = point_cloud2.read_points(msg, field_names=(
            'x', 'y', 'z', 'intensity', 't', 
            'reflectivity', 'ring', 'ambient', 'range'), 
            skip_nans=True)
        
        processed_points = []
        for p in pc_gen:
            # 坐标变换
            point_vec = np.array([p['x'], p['y'], p['z'], 1.0])
            transformed = np.dot(self.tf_matrix, point_vec)
            tx, ty, tz = transformed[:3]

            # 裁剪处理（基于变换后的坐标）
            if not (self.crop_limits['x'][0] <= tx <= self.crop_limits['x'][1] and
                    self.crop_limits['y'][0] <= ty <= self.crop_limits['y'][1] and
                    self.crop_limits['z'][0] <= tz <= self.crop_limits['z'][1]):
                continue
            
            # 构造新点数据（使用变换后的坐标）
            new_point = {
                'x': tx,
                'y': ty,
                'z': tz,
                'intensity': p['intensity'],
                't': p['t'],
                'reflectivity': p['reflectivity'],
                'ring': p['ring'],
                'ambient': p['ambient'],
                'range': p['range']
            }
            processed_points.append(new_point)

        # 找出 intensity > 3000 的点
        high_intensity_points = [p for p in processed_points if p['intensity'] > 2500]

        if high_intensity_points:
            # 提取 z 值
            z_values = [p['z'] for p in high_intensity_points]
            avgz = sum(z_values) / len(z_values)

            # 计算比率 r = (avgz - 0.4) / (1 - 0.4)
            r = (avgz - 0.4) / (1 - 0.4)

            self.get_logger().info(f"满载率={r:.3f}")
            # 发布一个文本 Marker 来显示比率
            marker = Marker()
            marker.header.frame_id = self.output_frame  # 设置你想显示的坐标系
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "volume_cal"
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position = Point(x=0.0, y=-0.4, z=2.0)  # 设置文本显示位置，调节这个位置以适合你的视图
            marker.scale.z = 0.5  # 设置字体大小

            # 设置文本内容
            marker.text = f"Load_rate:{r:.3f}"

            # 设置颜色
            marker.color.r = 1.0  # 红色
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # 不透明

            # 发布 Marker
            self.marker_pub.publish(marker)
            self.get_logger().info(f"发布比率 r={r:.3f} 的文本标记")
        else:
            self.get_logger().info("没有找到 intensity > 2500 的点")

        
        # 创建输出点云
        if processed_points:
            # 构造字段描述
            fields = [
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name='t', offset=20, datatype=PointField.UINT32, count=1),
                PointField(name='reflectivity', offset=24, datatype=PointField.UINT16, count=1),
                PointField(name='ring', offset=26, datatype=PointField.UINT16, count=1),
                PointField(name='ambient', offset=28, datatype=PointField.UINT16, count=1),
                PointField(name='range', offset=32, datatype=PointField.UINT32, count=1),
            ]


            
            # 构造点数据数组
            cloud_data = []
            for p in processed_points:
                cloud_data.append([
                    p['x'], p['y'], p['z'],
                    p['intensity'],
                    p['t'],
                    p['reflectivity'],
                    p['ring'],
                    p['ambient'],
                    p['range']
                ])
            
            # 创建并发布点云（保持原始坐标系）
            header = msg.header
            header.frame_id = self.output_frame  # 设置为变换后的坐标系
            new_cloud = point_cloud2.create_cloud(header, fields, cloud_data)
            self.publisher.publish(new_cloud)
            # self.get_logger().info(f"发布 {len(processed_points)} 个点", throttle_duration_sec=1)
        else:
            self.get_logger().warn("裁剪后无有效点云数据")

def main(args=None):
    rclpy.init(args=args)
    node = VolumeCal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()