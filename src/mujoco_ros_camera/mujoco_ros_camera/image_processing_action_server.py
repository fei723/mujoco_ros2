import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from ros_interfaces.action import ImageProcessing  # 替换为你的包名

class ImageProcessingActionServer(Node):
    def __init__(self):
        super().__init__('image_processing_action_server')
        self._action_server = ActionServer(
            self,
            ImageProcessing,
            'image_processing',
            self.execute_callback
        )
        self.bridge = CvBridge()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Starting image processing...')

        # 将接收到的 ROS 图像消息转换为 OpenCV 图像
        cv_image = self.bridge.imgmsg_to_cv2(goal_handle.request.image, desired_encoding='bgr8')

        # 转换为 HSV 颜色空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 定义红色的 HSV 范围（红色在 HSV 空间中不连续，需要两个范围）
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # 创建掩码
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 绘制边界框
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # 过滤掉小的轮廓
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 将处理后的 OpenCV 图像转换为 ROS 图像消息
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        result = ImageProcessing.Result()
        result.processed_image = processed_image_msg

        goal_handle.succeed()

        self.get_logger().info('Image processing completed.')
        return result

def main(args=None):
    rclpy.init(args=args)
    image_processing_action_server = ImageProcessingActionServer()
    rclpy.spin(image_processing_action_server)
    image_processing_action_server.destroy_node()
    rclpy.shutdown()