from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mi_robot_arm", package_name="mirobot_moveit").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)
    return LaunchDescription([
        Node(
            package='mujoco_ros_camera',
            executable='test_camera',
            output='screen'
        ),
        ld
        
    ])







# Node(
#             package='mujoco_ros_camera',
#             executable='test_show_node',
#             output='screen'
#         ),
        
# Node(
        #     package='joint_angle_subscriber',
        #     executable='joint_angle_subscriber',
        #     output='screen'
        # ),
        # Node(
        #     package='mujoco_ros_camera',
        #     executable='test_IK_serv',
        #     output='screen'
        # ),