#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>  // 添加必要的头文件
#include <map>
#include <vector>

class PIDController : public rclcpp::Node {
public:
    PIDController() : Node("pid_controller"), 
                    kp_(100.0), ki_(1.0), kd_(10.0), 
                    integral_limit_(50.0), last_time_(0.0) {
        
        // 订阅目标关节状态
        target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&PIDController::target_callback, this, std::placeholders::_1)
        );

        // 订阅当前关节状态
        current_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/arm_joint_angles", 10, 
            std::bind(&PIDController::current_callback, this, std::placeholders::_1)
        );

        // 创建力矩发布者
        torque_pub_ = create_publisher<sensor_msgs::msg::JointState>("/applied_torques", 10);

        // 初始化关节映射
        initialize_joint_mapping();
    }

private:
    // PID参数
    double kp_;
    double ki_;
    double kd_;
    double integral_limit_;
    std::vector<double> integral_;
    std::vector<double> last_error_;
    double last_time_;

    // 关节映射
    std::map<std::string, size_t> joint_map_;
    std::vector<std::string> joint_names_;

    // 数据存储
    sensor_msgs::msg::JointState target_joints_;
    sensor_msgs::msg::JointState current_joints_;
    std::mutex data_mutex_;  // 互斥锁声明

    // ROS接口
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_pub_;

    void initialize_joint_mapping() {
        // 初始化关节名称列表（根据实际模型修改）
        joint_names_ = {
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
            "joint7", "panda_finger_joint1", "panda_finger_joint2"
        };
        
        // 创建关节名称到索引的映射
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            joint_map_[joint_names_[i]] = i;
        }

        // 初始化积分项和历史误差
        integral_.resize(joint_names_.size(), 0.0);
        last_error_.resize(joint_names_.size(), 0.0);
    }

    void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> holder(data_mutex_);  // 修正拼写错误
        target_joints_ = *msg;
    }

    void current_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> holder(data_mutex_);  // 修正拼写错误
        current_joints_ = *msg;
    }

    void compute_pid() {
        std::lock_guard<std::mutex> holder(data_mutex_);  // 修正拼写错误
        
        // 获取当前时间
        double current_time = now().seconds();
        if (last_time_ == 0.0) {
            last_time_ = current_time;
            return;
        }

        double dt = current_time - last_time_;
        last_time_ = current_time;

        // 初始化输出力矩
        sensor_msgs::msg::JointState torque_msg;
        torque_msg.name = joint_names_;
        torque_msg.position.resize(joint_names_.size(), 0.0);

        // 遍历每个关节
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            std::string joint_name = joint_names_[i];

            // 获取目标位置和当前位置
            auto target_iter = std::find(target_joints_.name.begin(), target_joints_.name.end(), joint_name);
            auto current_iter = std::find(current_joints_.name.begin(), current_joints_.name.end(), joint_name);

            if (target_iter == target_joints_.name.end() || current_iter == current_joints_.name.end()) {
                RCLCPP_WARN(get_logger(), "Joint %s not found in messages", joint_name.c_str());
                continue;
            }

            size_t target_idx = std::distance(target_joints_.name.begin(), target_iter);
            size_t current_idx = std::distance(current_joints_.name.begin(), current_iter);

            double target_pos = target_joints_.position[target_idx];
            double current_pos = current_joints_.position[current_idx];

            // 计算误差
            double error = target_pos - current_pos;
            double de = (error - last_error_[i]) / dt;

            // 更新积分项
            integral_[i] += error * dt;
            integral_[i] = std::max(std::min(integral_[i], integral_limit_), -integral_limit_);

            // PID计算
            double torque = kp_ * error + ki_ * integral_[i] + kd_ * de;

            // 限制力矩输出
            double max_torque = 50.0;
            torque = std::max(std::min(torque, max_torque), -max_torque);

            torque_msg.position[i] = torque;
            last_error_[i] = error;
        }

        // 发布力矩
        torque_pub_->publish(torque_msg);
    }

public:
    void run() {
        // 创建定时器（100Hz）
        auto timer = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PIDController::compute_pid, this)
        );

        rclcpp::spin(this->get_node_base_interface());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<PIDController>();
    controller->run();
    rclcpp::shutdown();
    return 0;
}