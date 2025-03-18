import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
import mujoco
import numpy as np

class VideoSubscriberNode(Node):
    def __init__(self):
        super().__init__('video_subscriber_node')
        # 加载 MuJoCo 模型
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        self.model = mujoco.MjModel.from_xml_path(path)
        # 存储相机的字典
        self.camera_id_dict = {}
        self.image_subscribers = {}  # 用来存储彩色图像订阅者的字典
        self.depth_subscribers = {}  # 用来存储深度图像订阅者的字典

        # 生成相机 ID 字典
        self.generate_idx()

        # 动态创建彩色图像和深度图像订阅者
        self.create_subscribers()

        self.bridge = CvBridge()
        self.get_logger().info('Video Subscriber Node has been started.')

        # 启动 OpenCV 窗口
        for name in self.camera_id_dict.keys():
            cv2.namedWindow(f'Color Image from {name}', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(f'Depth Image from {name}', cv2.WINDOW_AUTOSIZE)

    def create_subscribers(self):
        for name in self.camera_id_dict.keys():
            # 彩色图像话题
            color_topic_name = f'image/{name}'
            self.get_logger().info(f"Subscribing to color topic: {color_topic_name}")
            self.image_subscribers[name] = self.create_subscription(
                Image,
                color_topic_name,
                lambda msg, cam_name=name: self.color_listener_callback(msg, cam_name),
                10
            )

            # 深度图像话题
            depth_topic_name = f'depth_image/{name}'
            self.get_logger().info(f"Subscribing to depth topic: {depth_topic_name}")
            self.depth_subscribers[name] = self.create_subscription(
                Image,
                depth_topic_name,
                lambda msg, cam_name=name: self.depth_listener_callback(msg, cam_name),
                10
            )

    def color_listener_callback(self, msg, name):
        try:
            # 将 ROS 彩色图像消息转换为 OpenCV 图像
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 显示彩色图像
            cv2.imshow(f'Color Image from {name}', cv_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing color image from {name}: {e}")

    def depth_listener_callback(self, msg, name):
        try:
            # 将 ROS 深度图像消息转换为 OpenCV 图像
            cv_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # 直接显示深度图像
            cv2.imshow(f'Depth Image from {name}', cv_depth_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing depth image from {name}: {e}")

    def generate_idx(self):
        for i in range(self.model.ncam):  # 遍历所有定义的相机
            camera_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            self.camera_id_dict[camera_name] = camera_id

def main(args=None):
    rclpy.init(args=args)
    video_subscriber_node = VideoSubscriberNode()

    try:
        # 创建多线程执行器
        executor = MultiThreadedExecutor()
        executor.add_node(video_subscriber_node)
        # 启动执行器
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        video_subscriber_node.destroy_node()
        cv2.destroyAllWindows()  # 关闭 OpenCV 窗口
        rclpy.shutdown()

