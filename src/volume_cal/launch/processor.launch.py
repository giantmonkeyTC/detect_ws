import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 包路径获取
    pkg_path = get_package_share_directory('volume_cal')
    
    return LaunchDescription([
        # 点云处理节点
        Node(
            package='volume_cal',
            executable='processor',
            name='volume_cal',
            parameters=[os.path.join(pkg_path, 'config', 'params.yaml')],
            output='screen'
        ),
        
        # 静态TF变换：map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base',
            arguments=[
                '--x', '0.0',  # X轴平移
                '--y', '0.0',  # Y轴平移
                '--z', '0.0',  # Z轴平移
                '--roll', '0.0',  # 横滚角
                '--pitch', '0.0',  # 俯仰角
                '--yaw', '0.0',  # 偏航角
                '--frame-id', 'map',
                '--child-frame-id', 'base_link'
            ],
            output='screen'
        ),
        
        # 静态TF变换：base_link -> os_sensor
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_sensor',
            arguments=[
                '--x', '-1.746215451773',   # 假设传感器前移0.5米
                '--y', '1.100357561283',
                '--z', '1.922401228849',   # 假设高度0.3米
                '--roll', '1.50905949',
                '--pitch', '-0.43424417',
                '--yaw', '1.04774539',
                '--frame-id', 'base_link',
                '--child-frame-id', 'os_sensor'
            ],
            output='screen'
        )
    ])