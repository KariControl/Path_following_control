from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'vehicle_sim'

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package=pkg_name,
        executable='vehicle_sim',
        name='vehicle_node',
        output='screen',
        parameters=[{
        'a_coe': 0.36,  # 1.0/2.778
        'b_coe': 0.1,  # 0.2778
        'diff_time': 0.01,  # サンプリング時間
        'vehicle_speed': 2.778,  # 初期車速
        'wheel_base': 1.0,  # ホイールベース
        }]
    )
    ld.add_action(node)
    return ld
