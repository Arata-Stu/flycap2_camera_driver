import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BayerToRgbConverter(Node):
    def __init__(self):
        super().__init__('bayer_to_rgb_converter')
        self.bridge = CvBridge()

        # サブスクライバ設定 (bayer_gbrg8形式の画像を受信)
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # トピック名
            self.listener_callback,
            10)
        
        # パブリッシャ設定 (rgb8形式の画像を配信)
        self.publisher = self.create_publisher(Image, 'rgb_image', 10)

    def listener_callback(self, msg):
        # ROS画像メッセージをOpenCV画像に変換
        try:
            bayer_image = self.bridge.imgmsg_to_cv2(msg, "bayer_gbrg8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # BayerGBをRGBに変換
        rgb_image = cv2.cvtColor(bayer_image, cv2.COLOR_BayerGB2BGR)

        # RGB画像をROS画像メッセージに変換
        try:
            rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            # 元のメッセージのヘッダをコピー
            rgb_image_msg.header = msg.header
            self.publisher.publish(rgb_image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = BayerToRgbConverter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
