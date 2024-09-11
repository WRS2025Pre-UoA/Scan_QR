import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 0.5秒ごとにパブリッシュ
        self.br = CvBridge()  # OpenCVの画像をROSメッセージに変換するためのブリッジ
        self.get_logger().info("Image publisher node has been started.")
    
    def timer_callback(self):
        img = cv2.imread('/path/to/your/image.jpg')  # パスを指定
        if img is None:
            self.get_logger().error("Image not found.")
            return

        img_msg = self.br.cv2_to_imgmsg(img, "bgr8")
        
        self.publisher_.publish(img_msg)
        self.get_logger().info("Publishing image...")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
