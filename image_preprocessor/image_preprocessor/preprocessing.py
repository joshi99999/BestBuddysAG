from sys import path
path.append("src/image_preprocessor/image_preprocessor")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from scaleDetection import ScaleDetector
import numpy as np

## @package image_preprocessor
# This package contains image preprocessing tools.

## A Preprocessor instance is used to create a ROS2 node for image preprocessing.
# The node subscribes the topic "camera_stream", 
# preprocesses incoming images from that topic 
# and publishes the preprocessed images on the topic "preprocessed_stream".
#
# The preprocessing includes:
# + Cutting out the belt area.
# + Transformation of the perspective.
# + Conversion to a binary image.

class Preprocessor(Node):

    ## Initializes a new Preprocessor instance.
    # @param queue The queue size of the camera subscription and the publishing of preprocessed images.
    # @param length The length (witdh) of the preprocessed images in pixels.
    # @param m Number of scale squares to the end of the belt area of interest, counted from the start of the scale.
    # @param n Number of scale squares to the beginning of the belt area of interest, counted from the start of the scale.

    def __init__(self, queue, length, m, n):
        super().__init__('preprocessor')
        self.__length = length
        self.__input = self.create_subscription(Image, 'camera_stream', self.preprocess, queue)
        self.__stream = self.create_publisher(Image, 'preprocessed_stream', queue)
        self.__detector = ScaleDetector(delta1=20, delta2=5, n=20, phi1=0.02, phi2=0.005, sigma1=0.02, sigma2=0.0005, m=20, count=20, accuracy=0.7, threshold=0.9)
        self.__bridge = CvBridge()
        self.__M = None
        self.__borders = None
        self.__n = n
        self.__l = length/(m-n)

    ## Preprocesses incoming images from the camera. 
    # + Detects the scale once and asks whether the detection was successful. 
    #   Repeats the detection, if the detected area was declined.
    #   For following images the detected area is kept.
    # + Cuts out the belt area and transforms it to a rectangle.
    # + Converts the transformed belt area to a binary image.
    # + Publishes the preprocessed image.
    # 
    # @param input_msg Image to be preprocessed.

    def preprocess(self, input_msg):
        image = self.__bridge.imgmsg_to_cv2(input_msg)
        gray = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(src=gray, ksize=(5,5), sigmaX=0)
        binary = cv2.threshold(src=gray, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]

        if self.__M is None:
            self.__borders = self.__detector.detectScale(img=binary)
            if self.__borders is None:
                self.get_logger().error('Scale detection failed.')
                return

            self.__M = cv2.getPerspectiveTransform(self.__borders[[0,1,0,1],[self.__n,self.__n,-1,-1]], np.float32([[0, -self.__l/2],[0, -3*self.__l/2],[self.__length, -self.__l/2],[self.__length, -3*self.__l/2]]))
            
            area = np.linalg.inv(self.__M) @ np.float32([[0,0,1],[self.__length,0,1],[self.__length,7*self.__l/2,1],[0,7*self.__l/2,1]]).T
            area = (area/area[2]).T[:,0:2].astype(np.int32)
            cv2.polylines(img=image, pts=[area], isClosed=True, color=(0,0,255), thickness=2)
            cv2.polylines(img=image, pts=[self.__borders[[0,1,1,0],[0,0,-1,-1]].astype(np.int32)], isClosed=True, color=(0,0,255), thickness=2)
            image = cv2.resize(src=image, dsize=(image.shape[1]//2, image.shape[0]//2))
            cv2.imshow("Area", image)
            cv2.waitKey(100)
            if 'y' != input("Has the conveyor belt been correctly detected?"):
                self.__M = None
                return
            cv2.destroyAllWindows()

        belt = cv2.warpPerspective(src=gray, M=self.__M, dsize=(self.__length, int(7*self.__l/2)))
        belt = cv2.threshold(src=belt, thresh=200, maxval=255, type=cv2.THRESH_BINARY)[1]
        cv2.imshow("Belt", belt)
        cv2.waitKey(1)
        output_msg = self.__bridge.cv2_to_imgmsg(belt, '8UC1')
        output_msg.header.stamp = input_msg.header.stamp
        self.__stream.publish(output_msg)
        self.get_logger().info('Image published.')

def main(args=None):
    rclpy.init(args=args)
    preprocessor = Preprocessor(3, 800, 20, 4)
    rclpy.spin(preprocessor)
    preprocessor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()