from sys import path
path.append("src/image_preprocessor/image_preprocessor")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from scaleDetection import ScaleDetector
import numpy as np


class Camera(Node):

    def __init__(self, queue, length):
        super().__init__('camera')
        self.length = length
        self.input = self.create_subscription(Image, 'camera_stream', self.preprocess, queue)
        self.stream = self.create_publisher(Image, 'preprocessed_stream', queue)
        self.detector = ScaleDetector(delta1=20, delta2=5, n=20, phi1=0.02, phi2=0.005, sigma1=0.02, sigma2=0.0005, m=20, count=20, accuracy=0.7)
        self.bridge = CvBridge()
        self.M = None

    def preprocess(self, input_msg):
        image = self.bridge.imgmsg_to_cv2(input_msg)
        gray = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(src=gray, ksize=(5,5), sigmaX=0)
        binary = cv2.threshold(src=gray, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]

        if self.M is None:
            corners = self.detector.detectScale(img=binary)
            if corners is None:
                self.get_logger().error('Scale detection failed.')
                return
            
            m, n = 20, 4
            l = self.length/(m-n)
            self.M = cv2.getPerspectiveTransform(corners, np.float32([[-n*l, -l/2],[-n*l, -3*l/2],[self.length, -l/2],[self.length, -3*l/2]]))
            
            area = np.linalg.inv(self.M) @ np.float32([[0,0,1],[self.length,0,1],[self.length,7*l/2,1],[0,7*l/2,1]]).T
            area = (area/area[2]).T[:,0:2].astype(np.int16)
            cv2.line(img=image, pt1=area[0], pt2=area[1],color=(0,0,255), thickness=2)
            cv2.line(img=image, pt1=area[1], pt2=area[2],color=(0,0,255), thickness=2)
            cv2.line(img=image, pt1=area[2], pt2=area[3],color=(0,0,255), thickness=2)
            cv2.line(img=image, pt1=area[3], pt2=area[0],color=(0,0,255), thickness=2)
            cv2.imshow("Area", image)
            cv2.waitKey(1)
            if 'y' != input("Has the conveyor belt been correctly detected?"):
                self.M = None
                return

        belt = cv2.warpPerspective(src=binary, M=self.M, dsize=self.shape)
        output_msg = self.bridge.cv2_to_imgmsg(belt)
        output_msg.header = input_msg.header
        self.stream.publish(output_msg)
        self.get_logger().info('Image published.')

def main(args=None):
    rclpy.init(args=args)
    camera = Camera(3, 800)
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()