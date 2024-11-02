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
        'kp': 1.2,  # 比例ゲイン 
        'ki': 3.0,  # 積分ゲイン
        'k2': 0.6,  # k2ゲイン
        'k3': 0.1,  # k3ゲイン
        'dt': 0.05,  # サンプリング時間
        'target_velocity': 2.0,  # 目標値
        'wheel_base': 1.0,  # ホイールベース
        }]
    )
    ld.add_action(node)
    return ld