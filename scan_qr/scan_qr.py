import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from scan_qr import qr_reader


class QRSubscriber(Node):

    def __init__(self):
        super().__init__('qr_subscriber')
        self.image_publisher = self.create_publisher(Image, 'qr_image', 10)
        self.value_publisher = self.create_publisher(String, 'qr_value', 10)
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
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv_image = qr_reader.image_resize(cv_image)

        original_image = cv_image.copy()
        processed_image, data = qr_reader.scan_qr(
            qr_reader.convert_image(cv_image), qr_reader.OPENCV, original_image)
        if data == []:
            print(f'{qr_reader.RED}読み取れませんでした{qr_reader.RESET_COLOR}')
        elif data[0] == "":
            print(f'{qr_reader.YELLOW}内容がわかりませんでした{qr_reader.RESET_COLOR}')
        else:
            text = ', '.join(data)
            print(f'{qr_reader.GREEN}内容:{text}{qr_reader.RESET_COLOR}')

        text = ','.join(data)
        if text != "":
            pub_data = String()
            pub_data.data = ','.join(data)
            self.value_publisher.publish(pub_data)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.image_publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)
            return

        while True:
            cv2.imshow('QR Result', processed_image)
            key = cv2.waitKey(10) & 0xff
            if key != 0xff or cv2.getWindowProperty('QR Result', cv2.WND_PROP_VISIBLE) == 0:
                cv2.destroyAllWindows()
                break


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
