import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # 订阅关节状态
        self.target_sub = self.create_subscription(
            JointState, '/joint_states', self.target_callback, 10)
        self.current_sub = self.create_subscription(
            JointState, '/arm_joint_angles', self.current_callback, 10)
        
        # 发布控制力矩
        self.torque_pub = self.create_publisher(JointState, '/applied_torques', 10)

        # 初始化关节列表
        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6',
            'joint7', 'panda_finger_joint1', 'panda_finger_joint2'
        ]

        # 初始化PID参数（每个关节独立）
        self.params = {
            joint: {
                'kp': 40.0,
                'ki': 5.0,
                'kd': 8.0,
                'integral_limit': 5.0,
                'max_torque': 50
            } for joint in self.joint_names
        }

        # 声明动态参数
        self.declare_parameters(
            namespace='',
            parameters=[
                (f'{joint}/kp', self.params[joint]['kp']) for joint in self.joint_names
            ] + [
                (f'{joint}/ki', self.params[joint]['ki']) for joint in self.joint_names
            ] + [
                (f'{joint}/kd', self.params[joint]['kd']) for joint in self.joint_names
            ] + [
                (f'{joint}/integral_limit', self.params[joint]['integral_limit']) for joint in self.joint_names
            ] + [
                (f'{joint}/max_torque', self.params[joint]['max_torque']) for joint in self.joint_names
            ]
        )

        # 注册参数回调
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 初始化状态存储
        self.target_joints = None
        self.current_joints = None
        self.integral = {joint: 0.0 for joint in self.joint_names}
        self.last_error = {joint: 0.0 for joint in self.joint_names}
        self.last_time = self.get_clock().now()

        # 创建定时器
        self.timer = self.create_timer(0.01, self.compute_pid)  # 替换为 create_timer

    def target_callback(self, msg):
        self.target_joints = msg

    def current_callback(self, msg):
        self.current_joints = msg

    def parameter_callback(self, params):
        for param in params:
            # 解析关节名称和参数类型
            parts = param.name.split('/')
            if len(parts) != 2:
                continue
            
            joint, param_type = parts
            if joint not in self.joint_names:
                continue
            
            # 更新参数值
            self.params[joint][param_type] = param.value
            self.get_logger().info(f"Updated {joint} {param_type} to {param.value}")
        
        return SetParametersResult(successful=True)

    def compute_pid(self):
        if self.target_joints is None or self.current_joints is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        torque_msg = JointState()
        torque_msg.name = self.joint_names
        torque_msg.position = [0.0] * len(self.joint_names)

        for i, joint in enumerate(self.joint_names):
            try:
                target_idx = self.target_joints.name.index(joint)
                current_idx = self.current_joints.name.index(joint)
                target_pos = self.target_joints.position[target_idx]
                current_pos = self.current_joints.position[current_idx]

                # 获取当前关节参数
                kp = self.params[joint]['kp']
                ki = self.params[joint]['ki']
                kd = self.params[joint]['kd']
                integral_limit = self.params[joint]['integral_limit']
                max_torque = self.params[joint]['max_torque']

                # 计算误差
                error = target_pos - current_pos
                de = (error - self.last_error[joint]) / dt

                # 更新积分项
                self.integral[joint] += error * dt
                self.integral[joint] = max(min(self.integral[joint], integral_limit), -integral_limit)

                # PID计算
                torque = kp * error + ki * self.integral[joint] + kd * de

                # 限制力矩
                torque = max(min(torque, max_torque), -max_torque)

                torque_msg.position[i] = torque
                self.last_error[joint] = error

            except ValueError:
                self.get_logger().warn(f"Joint {joint} not found in messages")

        self.torque_pub.publish(torque_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
