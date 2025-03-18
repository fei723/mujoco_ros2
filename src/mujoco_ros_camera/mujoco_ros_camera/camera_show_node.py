import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from ament_index_python.packages import get_package_share_directory
import mujoco


class VideoSubscriberNode(Node):
    def __init__(self):
        super().__init__('video_subscriber_node')
        # 加载 MuJoCo 模型
        path = get_package_share_directory('mujoco_ros_camera') + '/xml/MI_arm/MI_robot_ori.xml'
        self.model = mujoco.MjModel.from_xml_path(path)
        # 存储相机的字典
        self.camera_id_dict = {}
        self.image_subscribers = {}  # 用来存储订阅者的字典

        # 生成相机 ID 字典
        self.generate_idx()

        # 动态创建订阅者
        self.create_subscribers()

        self.bridge = CvBridge()
        self.get_logger().info('Video Subscriber Node has been started.')

        # 启动 OpenCV 窗口
        for name in self.camera_id_dict.keys():
            cv2.namedWindow(f'Received Image from {name}', cv2.WINDOW_AUTOSIZE)

        # 启动独立的窗口事件处理线程
        self.window_thread = threading.Thread(target=self.process_window_events)
        self.window_thread.daemon = True  # 随主线程退出
        self.window_thread.start()

    def create_subscribers(self):
        for name in self.camera_id_dict.keys():
            topic_name = f'image/{name}'
            print(f"Subscribing to topic: {topic_name}")
            self.image_subscribers[name] = self.create_subscription(
                Image,
                topic_name,
                self.get_callback(name),
                10
            )

    def get_callback(self, name):
        def callback(msg):
            self.listener_callback(msg, name)
        return callback

    def listener_callback(self, msg, name):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 显示图像（非阻塞）
            cv2.imshow(f'Received Image from {name}', cv_img)
        except Exception as e:
            self.get_logger().error(f"Error processing image from {name}: {e}")

    def generate_idx(self):
        for i in range(self.model.ncam):  # 遍历所有定义的相机
            camera_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            self.camera_id_dict[camera_name] = camera_id

    def process_window_events(self):
        # 独立线程处理窗口事件
        while rclpy.ok():
            cv2.waitKey(1)  # 仅处理事件，不阻塞主线程


def main(args=None):
    rclpy.init(args=args)
    video_subscriber_node = VideoSubscriberNode()

    try:
        rclpy.spin(video_subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        video_subscriber_node.destroy_node()
        cv2.destroyAllWindows()  # 关闭 OpenCV 窗口
        rclpy.shutdown()


