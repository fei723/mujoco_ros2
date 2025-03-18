import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取 MoveIt2 配置包的路径
    moveit_config_package = "mirobot_moveit"  # 替换为你的 MoveIt2 配置包名
    moveit_config_path = get_package_share_directory(moveit_config_package)

    # 获取 URDF 文件路径
    urdf_file = os.path.join(moveit_config_path, "config", "mi_robot_arm.urdf.xacro")  # 替换为你的 URDF 或 Xacro 文件

    # 获取运动学配置文件路径
    kinematics_file = os.path.join(moveit_config_path, "config", "kinematics.yaml")

    # 获取 SRDF 文件路径
    srdf_file = os.path.join(moveit_config_path, "config", "mi_robot_arm.srdf")  # 替换为你的 SRDF 文件

    # 声明启动参数
    db_arg = DeclareLaunchArgument(
        "db",
        default_value="false",
        description="By default, we do not start a database (it can be large)"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="By default, start RViz"
    )

    # 启动机器人状态发布器
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": open(urdf_file).read()
        }]
    )

    # 启动 MoveGroup 节点
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_path, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "robot_description_file": urdf_file,
            "robot_description_semantic_file": srdf_file,
            "kinematics_yaml": kinematics_file,
            "use_sim_time": "false",
            "use_rviz": LaunchConfiguration("rviz"),
            "use_db": LaunchConfiguration("db")
        }.items()
    )

    # 启动 RViz
    rviz = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(moveit_config_path, "config", "moveit.rviz")]
    )

    return LaunchDescription([
        db_arg,
        rviz_arg,
        robot_state_publisher,
        move_group,
        rviz
    ])
    