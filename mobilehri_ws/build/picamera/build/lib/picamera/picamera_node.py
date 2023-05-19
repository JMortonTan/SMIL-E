import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class picamera(Node):

    def __init__(self):
        super().__init__('PIimage_publisher')
        self.cap = cv2.VideoCapture(0)
        self.image_pub = self.create_publisher(Image, '/picamera/image', 10)
        timer_period = 1.0/30.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # some settings that may be useful when you are outdoors
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 2)

        if not self.cap.isOpened():
            print("cannot open camera")
            exit()
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        # image = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")   
        if ret == True:
            image = self.bridge.cv2_to_imgmsg(frame)
            self.image_pub.publish(image)

def main(args=None):
    rclpy.init(args=args)
    camera_node = picamera()
    rclpy.spin(camera_node)
    camera_node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
