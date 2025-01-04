from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取配置文件路径
    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "pgo.yaml"]
    )

    # 添加 rviz 配置
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    # Original FastLIO node
    fastlio_node = Node(
        package='fastlio2',
        namespace='fastlio2',  # 添加命名空间
        executable='lio_node',
        name='lio_node',
        parameters=[{"config_path": config_path.perform(launch.LaunchContext())}],
        output='screen'
    )

    # PGO node
    pgo_node = Node(
        package='fastlio2',
        namespace='fastlio2',  # 添加命名空间
        executable='lio_pgo_node',
        name='lio_pgo_node',
        parameters=[{"config_path": pgo_config_path.perform(launch.LaunchContext())}],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        namespace='fastlio2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg.perform(launch.LaunchContext())],
    )

    return LaunchDescription([
        fastlio_node,
        pgo_node,
        rviz_node
    ])