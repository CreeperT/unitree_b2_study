from launch import LaunchDescription
from launch_ros.actions import Node
#封装终端指令相关类--------------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
#参数声明与获取-----------------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
#文件包含相关-------------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
#分组相关----------------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
#事件相关----------------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
#获取功能包下share目录路径-------------
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory("pointcloud")
    config_path = os.path.join(pkg_share_dir, 'config', 'parameters.yaml')
    rviz_path = os.path.join(pkg_share_dir, 'rviz', 'pointcloud_pub.rviz')

    action_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0.0', '0.0', '0.0',
                   '0', '0', '0', '1', 
                   'map', 'rslidar']
    )

    action_pointcloud_pub_node = Node(
        package="pointcloud",
        executable="pointcloud_pub",
        parameters=[config_path]
    )

    action_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_path]
    )
    
    return LaunchDescription([
        action_static_tf_node,
        action_pointcloud_pub_node,
        action_rviz_node
    ])