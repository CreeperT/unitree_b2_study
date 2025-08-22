from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
#封装终端指令相关类--------------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
#参数声明与获取-----------------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import Launchconfiguration
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
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_pkg_path = get_package_share_directory('b2_description')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'b2_description.urdf')
    # 如果使用其他的机器人模型
    model = DeclareLaunchArgument(
        name="model_urdf_path",
        default_value=str(default_urdf_path)
    )
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model_urdf_path")]))
    # 如果有第三方插件使用关节，修改参数不发布默认关节状态
    use_joint_state_publisher = DeclareLaunchArgument(
        name="use_joint_state_publisher",
        default_value="true"
    )
    # robot_state_publisher
    ## 加载b2的urdf文件
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    # joint_state_publisher
    ## 发布关节状态
    joint_stat_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher"))
    )
    return LaunchDescription([
        model,
        use_joint_state_publisher,
        robot_state_publisher,
        joint_stat_publisher
    ])