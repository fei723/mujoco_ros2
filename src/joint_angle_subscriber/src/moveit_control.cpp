#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc, char** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建一个节点
    auto node = rclcpp::Node::make_shared("moveit_control_node");

    // 创建 MoveGroupInterface 对象，指定规划组名称
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm_hand");

    // 设置规划的参考坐标系
    move_group.getPlanningFrame();

    // 设置目标关节角度
    std::vector<double> joint_goal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(joint_goal);

    // 进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Planning successful! Executing plan...");
        // 执行规划路径
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}