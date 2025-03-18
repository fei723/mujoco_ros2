import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
# 添加Ctrl+C处理
import signal


import mujoco
import mujoco.viewer

import numpy as np
import cv2

import threading
import time


class asset_init():
    def __init__(self):
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.01#控制仿真时间


class mujoco_publisher(Node):

    def __init__(self):
        super().__init__('mujoco_publisher')
        # 创建图像发布者
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(path)


class MujocoCameraPublisher(Node):
    """
        用来初始化mujoco model和data 以及发布图像节点的类
    """
    def __init__(self):
        super().__init__('mujoco_camera_publisher')
        # 创建图像发布者
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'

        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(path)

        self.camera_id_dict = {}
        self.generate_idx()

        self.image_publishers = {}  # 用来存储彩色图像发布者的字典
        self.depth_publishers = {}  # 用来存储深度图像发布者的字典

        for name, id in self.camera_id_dict.items():
            color_topic_name = f'image/{name}'  # 构造彩色图像话题名称
            depth_topic_name = f'depth_image/{name}'  # 构造深度图像话题名称
            self.image_publishers[name] = self.create_publisher(Image, color_topic_name, 10)
            self.depth_publishers[name] = self.create_publisher(Image, depth_topic_name, 10)

        self.bridge = CvBridge()
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "end_effector_camera")

        # 用于保存最新图像
        self.latest_image = {}
        self.latest_depth_image = {}
        self.lock = threading.Lock()

    def publish_camera_image(self, image, depth_image):
        while rclpy.ok():
            with self.lock:
                self.latest_image = image.copy()
                self.latest_depth_image = depth_image.copy()
                if self.latest_image and self.latest_depth_image:
                    for name, id in self.camera_id_dict.items():
                        if id in self.latest_image:  # 检查彩色图像键是否存在
                            try:
                                # 将彩色图像转换为 ROS 消息并发布
                                img_msg = self.bridge.cv2_to_imgmsg(self.latest_image[id], encoding='bgr8')
                                self.image_publishers[name].publish(img_msg)
                            except Exception as e:
                                self.get_logger().error(f"Error publishing color image from {name}: {e}")
                        if id in self.latest_depth_image:  # 检查深度图像键是否存在
                            try:
                                # 将深度图像转换为 ROS 消息并发布
                                depth_img_msg = self.bridge.cv2_to_imgmsg(self.latest_depth_image[id], encoding='mono16')
                                self.depth_publishers[name].publish(depth_img_msg)
                            except Exception as e:
                                self.get_logger().error(f"Error publishing depth image from {name}: {e}")
            time.sleep(0.010)  # 每30毫秒发布一次，大约30帧每秒

    def generate_idx(self):
        for i in range(self.model.ncam):  # 遍历所有定义的相机
            camera_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            self.camera_id_dict[camera_name] = camera_id
        print(self.camera_id_dict)



    def get_camera_intrinsics(self, camera_name, width, height):
        """
            获取相机内参参数函数
        """
        # 获取指定相机的索引
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)

        # 获取相机的垂直视野角（单位：度）
        v_fov = self.model.cam_fovy[camera_id]

        # 将垂直视野角转换为弧度
        v_fov_rad = np.radians(v_fov)

        # 计算焦距
        f_y = height / (2 * np.tan(v_fov_rad / 2))
        f_x = f_y  # 假设纵横比为 1

        # 计算主点坐标
        c_x = width / 2
        c_y = height / 2

        # 构建内参矩阵
        K = np.array([
            [f_x, 0, c_x],
            [0, f_y, c_y],
            [0, 0, 1]
        ], dtype=np.float32)

        return K

class render_image():
    """
        用来渲染的类，使用多线程，为了将渲染和主视图线程分隔开来，这样避免线程阻塞导致的程序缓慢
    """
    def __init__(self):
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(path)
        self.camera_id_dict = {}
        self.renderers = {}
        self.renderers_depth = {}
        self.generate_idx()
        self.lock = threading.Lock()
        self.image = {}
        self.depth_image = {}
        self.data = None

    def render(self, data):
        while rclpy.ok():
            with self.lock:
                if data is not None:
                    for name, id in self.camera_id_dict.items():
                        self.renderers[id].update_scene(data, camera=name)
                        img = self.renderers[id].render()
                        img_cv = np.array(img)
                        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_RGB2BGR)  # 转换颜色格式
                        cv2.imshow(name + 'View', img_cv)
                        cv2.waitKey(1)  # 处理图像窗口事件
                        self.image[id] = img_cv

    def render_depth(self, data):
        while rclpy.ok():
            with self.lock:
                if data is not None:
                    for name, id in self.camera_id_dict.items():
                        self.renderers_depth[id].enable_depth_rendering()
                        self.renderers_depth[id].update_scene(data, camera=name)
                        depth = self.renderers_depth[id].render()
                        # 不进行颜色映射，直接使用原始深度数据
                        depth = (depth * 65535).astype(np.uint16)  # 将深度数据转换为 16 位无符号整数
                        # cv2.imshow(name + ' Depth View', depth)
                        # cv2.waitKey(1)  # 处理图像窗口事件
                        self.depth_image[id] = depth

    def generate_idx(self):
        for i in range(self.model.ncam):  # 遍历所有定义的相机
            camera_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            self.camera_id_dict[camera_name] = camera_id
            self.renderers[camera_id] = mujoco.Renderer(self.model, width=1280, height=720)
            self.renderers_depth[camera_id] = mujoco.Renderer(self.model, width=1280, height=720)

    def update_data(self, data):
        with self.lock:
            self.data = data


class JointStatePublisher(Node):
    """
        用于发布机械臂关节角度的类
    """

    def __init__(self):
        super().__init__('arm_joint_angle_publisher')
        # 创建关节状态发布者
        self.joint_state_pub = self.create_publisher(JointState, 'arm_joint_angles', 10)
        # 加载 MuJoCo 模型
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        self.model = mujoco.MjModel.from_xml_path(path)

        # 获取关节名称
        self.joint_names = []
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_names.append(joint_name)
        self.joint_names = [ 'joint1',
                'joint2',
                'joint3',
                'joint4',
                'joint5',
                'joint6',
                'joint7',
                'panda_finger_joint1',
                'panda_finger_joint2'
                ]
    def publish_joint_angles(self, data):
        while rclpy.ok():
            # 创建 JointState 消息
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            # joint_state_msg.position = data.qpos.tolist()  # 获取关节角度
            for  name in self.joint_names:
                idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                joint_state_msg.position.append(data.qpos[idx])
            joint_angle_dict = dict(zip(self.joint_names, joint_state_msg.position))

            # 显示关节角度信息
            angle_info = "Joint Angles:\n"
            for name, angle in joint_angle_dict.items():
                angle_info += f"{name}: {angle:.4f} radians\n"
            # self.get_logger().info(angle_info)

            # 发布关节状态消息
            self.joint_state_pub.publish(joint_state_msg)
            # self.get_logger().info('Published joint angles')
            time.sleep(0.010)  # 每10毫秒发布一次


class JointStateSubscriber(Node):
    """
    接收MoveIt关节角度指令并控制仿真的类
    """
    def __init__(self):
        super().__init__('moveit_joint_subscriber')
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        self.model = mujoco.MjModel.from_xml_path(path)
        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]

        self.joint_positions = np.zeros(self.model.njnt)
        self.lock = threading.Lock()
        self.latest_joint_data = {
            'name': [],
            'position': [],
            'velocity': []
        }
        # 订阅MoveIt关节指令话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # MoveIt默认指令话题
            self.joint_command_callback,
            10
        )
        self.get_logger().info("MoveIt joint subscriber initialized")

    def joint_command_callback(self, msg):
        """
        处理MoveIt关节指令的回调函数
        """
        try:
            # 检查关节名称匹配
            # if set(msg.name) != set(self.joint_names):
            #     self.get_logger().error("Joint names mismatch between MoveIt and Mujoco")
            #     return
            with self.lock:
                self.latest_joint_data['name'] = msg.name[:]
                self.latest_joint_data['position'] = msg.position[:]
                self.latest_joint_data['velocity'] = msg.velocity[:]
        except Exception as e:
            self.get_logger().error(f"Error processing joint command: {str(e)}")

class TorqueSubscriber(Node):
    """
    接收外部力矩控制指令的类
    """
    def __init__(self):
        super().__init__('torque_subscriber')
        # 加载模型获取关节信息
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        self.model = mujoco.MjModel.from_xml_path(path)

        self.actuator_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(self.model.nu)
        ]
        # 预获取执行器索引
        self.actuator_indices = {name: i for i, name in enumerate(self.actuator_names)}

        self.lock = threading.Lock()
        # 使用执行器数量来初始化存储最新力矩的数组
        self.latest_torque = np.zeros(self.model.nu)

        # 订阅力矩指令话题
        self.subscription = self.create_subscription(
            JointState,
            '/applied_torques',  # 力矩话题名称
            self.torque_callback,
            10
        )
        self.get_logger().info("Torque subscriber initialized")

    def torque_callback(self, msg):
        """
        力矩指令回调函数
        """
        try:
            with self.lock:
                # 初始化力矩数组
                self.latest_torque.fill(0.0)

                # 遍历消息中的每个关节
                for name, torque in zip(msg.name, msg.position):
                    # 获取执行器索引
                    actuator_idx = self.actuator_indices.get(name, -1)
                    if actuator_idx != -1:
                        # 限制力矩范围（示例：-50到50 Nm）
                        max_torque = 50.0
                        self.latest_torque[actuator_idx] = np.clip(torque, -max_torque, max_torque)
                    else:
                        self.get_logger().warn(f"Actuator not found for joint: {name}")

        except Exception as e:
            self.get_logger().error(f"Error processing torque command: {str(e)}")

    def apply_torques(self, data):
        """
        将力矩应用到仿真模型
        """
        with self.lock:
            # 直接设置控制力矩
            data.ctrl[:] = self.latest_torque

def main(args=None):
    rclpy.init()
    asset = asset_init()
    camera_publisher = MujocoCameraPublisher()
    image_render = render_image()
    joint_state_publisher = JointStatePublisher()
    joint_command = JointStateSubscriber()
    torque_rec = TorqueSubscriber()
    mujoco.mj_step(asset.model, asset.data)
 
    with mujoco.viewer.launch_passive(asset.model, asset.data) as viewer:
        start = time.time()
        # 开启一个线程来进行多视角的渲染(rgb)
        render_thread = threading.Thread(target=image_render.render, args=(asset.data,))
        render_thread.start()
        # 开启一个线程来进行多视角的渲染(depth)
        d_render_thread = threading.Thread(target=image_render.render_depth, args=(asset.data,))
        d_render_thread.start()
        # 开启一个线程来发布图像
        publishing_thread = threading.Thread(target=camera_publisher.publish_camera_image,
                                             args=(image_render.image, image_render.depth_image,))
        publishing_thread.start()
        # 开启一个线程来发布关节角度
        joint_state_thread = threading.Thread(target=joint_state_publisher.publish_joint_angles, args=(asset.data,))
        joint_state_thread.start()

        while rclpy.ok():
            step_start = time.time()
            rclpy.spin_once(joint_command)
            # for name, pos in zip(joint_command.latest_joint_data['name'], joint_command.latest_joint_data['position']):
            #     idx = mujoco.mj_name2id(asset.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            #     asset.data.qpos[idx] = pos

            rclpy.spin_once(torque_rec)  # 新增

            # 应用力矩到模型
            torque_rec.apply_torques(asset.data)  # 新增

            # 进行一步仿真
            mujoco.mj_step(asset.model, asset.data)

            
                # print(f"{name}:{pos}")
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(asset.data.time % 2)

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = asset.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            
            

    
    # 确保在退出时清理线程
    publishing_thread.join()
    render_thread.join()
    d_render_thread.join()
    # joint_state_thread.join()

    image_render.close_renderers()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # 关闭所有 OpenCV 窗口


