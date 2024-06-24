import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from .. import qr_reader

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
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        qr_code_detector = cv2.QRCodeDetector()

        # QRコードを検出
        data, bboxes, _ = qr_code_detector.detectAndDecode(cv_image)

        # QRコードが検出された場合
        if bboxes is not None:
            # QRコードの内容を出力
            print(f'QRコードの内容: {data}')
            bbox=bboxes[0]

            if len(bbox) == 4 and all(len(box) == 2 for box in bbox):
                for i in range(len(bbox)):
                    # bboxの点を整数に変換
                    point1 = tuple(map(int,bbox[i]))
                    point2 = tuple(map(int, bbox[(i + 1) % len(bbox)]))

                    # 線を引く
                    cv2.line(cv_image, point1, point2, (0, 255, 0), 2)
                
                # QRコードの内容を画像に描画
                cv2.putText(cv_image, data, (bbox[0][0].astype(int), bbox[0][1].astype(int) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
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