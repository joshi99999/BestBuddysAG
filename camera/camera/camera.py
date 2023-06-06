from sys import path
path.append("src/camera/camera")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from scaleDetection import ScaleDetector
from time import time
from os import getcwd
from numpy import float32


class Camera(Node):

    def __init__(self, framerate):
        super().__init__('camera')
        self.stream = self.create_publisher(Image, 'camera_stream', 5)
        self.timer = self.create_timer(1/framerate, self.main)
        #self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.detector = ScaleDetector(delta1=20, delta2=5, n=20, phi1=0.02, phi2=0.005, m=20, count=20, accuracy=0.5)
        self.shape = (1000, 200)
        self.bridge = CvBridge()
        self.i = 1

    def main(self):
        #ret, original = self.camera.read()
        timestamp = time()
        #if not ret:
        #    return
        
        self.i = self.i + 1 if self.i < 12 else 1

        print(getcwd())

        original = cv2.imread("src/camera/images/image" + str(self.i) + ".jpg")
        
        gray = cv2.cvtColor(src=original, code=cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(src=gray, ksize=(5,5), sigmaX=0)
        binary = cv2.threshold(src=gray, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]

        corners = self.detector.detectScale(img=binary)
        if corners is None:
            return
        
        corners[0] += 9*(corners[0]-corners[1])//2
        corners[3] += 9*(corners[3]-corners[2])//2

        print(original.shape)

        M = cv2.getPerspectiveTransform(corners, float32([[self.shape[0], self.shape[1]], [self.shape[0], 0], [0, 0], [0, self.shape[1]]]))
        belt = cv2.warpPerspective(src=original, M=M, dsize=self.shape)
        cv2.imshow("Belt", belt)
        cv2.waitKey(1)
        msg = self.bridge.cv2_to_imgmsg(belt)
        msg.header.stamp.sec = int(timestamp)
        msg.header.stamp.nanosec = int((timestamp-msg.header.stamp.sec)*1000000000)
        self.stream.publish(msg)
        self.get_logger().info(str(int(timestamp*1000)) + ' - Publishing Image')

def main(args=None):
    rclpy.init(args=args)
    camera = Camera(1)
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()