import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from pathlib import Path
import sys
sys.path.append(str(Path(__file__).resolve().parent.parent))

import qr_reader

class QRSubscriber(Node):

    def __init__(self):
        super().__init__('qr_subscriber')
        self.publisher_ = self.create_publisher(Image, 'qr_image', 10)
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # ROS2のsensor_msgs/Image型からOpenCVで処理適量にcv::Mat型へ変換する。
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print(e)
            return
        
        cv_image=qr_reader.scan_qr(cv_image)
        
        try:
            ros_image=self.bridge.cv2_to_imgmsg(cv_image,'rgb8')
            self.publisher_.publish(ros_image)
        except CvBridgeError as e:
            print(e)
            return


def main(args=None):
    rclpy.init(args=args)

    qr_subscriber = QRSubscriber()

    rclpy.spin(qr_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qr_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()