from launch import LaunchDescription
from launch_ros.actions import Node
#封装终端指令相关类--------------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
#参数声明与获取-----------------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
#文件包含相关-------------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#分组相关----------------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
#事件相关----------------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
#获取功能包下share目录路径-------------
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition


"""
launch文件集成功能：
    1.机器人模型可视化
    2.速度消息桥接
    3.里程计消息发布、广播里程计坐标变换、发布关节状态信息
    4.
    5.
"""

def generate_launch_description():
    b2_description_pkg = get_package_share_directory("b2_description")
    b2_description_launch_path = os.path.join(b2_description_pkg, "launch", "display.launch.py")
    b2_driver_pkg = get_package_share_directory("b2_driver")
    rviz_cfg_path = os.path.join(b2_driver_pkg, "rviz", "display.rviz")
    # rviz开关
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true"
    )

    # 1.机器人模型可视化
    action_b2_description_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(launch_file_path=b2_description_launch_path))
    ## 包含rviz2
    action_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg_path],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )
    ## 雷达坐标系映射
    action_lidar_shine_upon = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "lidar_link", "--child-frame-id", "utlidar_lidar"]
    )

    # 2.速度消息桥接
    action_twist_bridge = Node(
        package="b2_twist_bridge",
        executable="twist_bridge"
    )
    
    # 3.里程计消息发布、广播里程计坐标变换、发布关节状态信息
    action_driver = Node(
        package="b2_driver",
        executable="driver"
    )

    return LaunchDescription([
        use_rviz,
        action_b2_description_launch,
        action_rviz2,
        action_lidar_shine_upon,
        action_twist_bridge,
        action_driver
    ])