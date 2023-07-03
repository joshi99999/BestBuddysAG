from sys import path
path.append("src/image_preprocessor/image_preprocessor")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from scaleDetection import ScaleDetector
import numpy as np


class Preprocessor(Node):

    def __init__(self, queue, length, m, n):
        super().__init__('preprocessor')
        self.length = length
        self.input = self.create_subscription(Image, 'camera_stream', self.preprocess, queue)
        self.stream = self.create_publisher(Image, 'preprocessed_stream', queue)
        self.detector = ScaleDetector(delta1=20, delta2=5, n=20, phi1=0.02, phi2=0.005, sigma1=0.02, sigma2=0.0005, m=20, count=20, accuracy=0.7, threshold=0.9)
        self.bridge = CvBridge()
        self.M = None
        self.borders = None
        self.n = n
        self.l = length/(m-n)
        self.errors = 0

    def preprocess(self, input_msg):
        image = self.bridge.imgmsg_to_cv2(input_msg)
        gray = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(src=gray, ksize=(5,5), sigmaX=0)
        binary = cv2.threshold(src=gray, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]

        if self.M is None:
            self.borders = self.detector.detectScale(img=binary)
            if self.borders is None:
                self.get_logger().error('Scale detection failed.')
                return

            self.M = cv2.getPerspectiveTransform(self.borders[[0,1,0,1],[self.n,self.n,-1,-1]], np.float32([[0, -self.l/2],[0, -3*self.l/2],[self.length, -self.l/2],[self.length, -3*self.l/2]]))
            
            area = np.linalg.inv(self.M) @ np.float32([[0,0,1],[self.length,0,1],[self.length,7*self.l/2,1],[0,7*self.l/2,1]]).T
            area = (area/area[2]).T[:,0:2].astype(np.int32)
            cv2.polylines(img=image, pts=[area], isClosed=True, color=(0,0,255), thickness=2)
            cv2.polylines(img=image, pts=[self.borders[[0,1,1,0],[0,0,-1,-1]].astype(np.int32)], isClosed=True, color=(0,0,255), thickness=2)
            image = cv2.resize(src=image, dsize=(image.shape[1]//2, image.shape[0]//2))
            cv2.imshow("Area", image)
            cv2.waitKey(100)
            if 'y' != input("Has the conveyor belt been correctly detected?"):
                self.M = None
                return
            cv2.destroyAllWindows()

        if not self.detector.checkScale():
            self.errors += 1
            self.get_logger().error('Image invalid.')
            if 10 < self.errors:
                self.destroy_node()
                rclpy.shutdown()
            return
        self.errors = 0

        belt = cv2.warpPerspective(src=gray, M=self.M, dsize=(self.length, int(7*self.l/2)))
        belt = cv2.threshold(src=belt, thresh=200, maxval=255, type=cv2.THRESH_BINARY)[1]
        cv2.imshow("Belt", belt)
        cv2.waitKey(1)
        output_msg = self.bridge.cv2_to_imgmsg(belt, '8UC1')
        output_msg.header.stamp = input_msg.header.stamp
        self.stream.publish(output_msg)
        self.get_logger().info('Image published.')

def main(args=None):
    rclpy.init(args=args)
    preprocessor = Preprocessor(3, 800, 20, 4)
    rclpy.spin(preprocessor)
    preprocessor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()