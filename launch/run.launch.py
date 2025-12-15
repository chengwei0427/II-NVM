from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "II_NVM"

def generate_launch_description():
    
    # 获取project功能包路径
    project_pkg = FindPackageShare(PACKAGE_NAME)
    config_file_path = '/home/cc/ros2_ws/src/II-NVM/config/'

    # 配置project节点 - 移除不兼容的命令行参数
    project_node = Node(
        package=PACKAGE_NAME,
        executable='II_NVM',
        name='II_NVM',
        output='screen',
        parameters=[{
            'config_file_path': config_file_path
        }]
    )
    
    # 配置rviz2节点（ROS2中rviz更名为rviz2）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='II_NVM',
        arguments=[
            '-d', 
            PathJoinSubstitution([project_pkg, 'launch', 'II_NVM.rviz'])
        ]
    )
    
    return LaunchDescription([
        project_node,
        rviz_node
    ])