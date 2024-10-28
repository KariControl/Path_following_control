from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'path_following'

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package=pkg_name,
        executable='path_following',
        name='path_following',
        output='screen',
        parameters=[{
        'kp': 0.766527,  # 比例ゲイン 
        'ki': 0.00,  # 積分ゲイン
        'kd': 0.728840,  # 微分ゲイン
        'dt': 0.1,  # サンプリング時間
        'set_point': 10.0,  # 目標値
        }]
    )
    ld.add_action(node)
    return ld