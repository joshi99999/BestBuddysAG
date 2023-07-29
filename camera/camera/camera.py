import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from time import time, sleep

## @package camera
# This package contains image capturing tools.

## A Camera instance is used to create a ROS2 node for image caprturing.
# The node captures images from a video device and publishes them on the topic "camera_stream".

class Camera(Node):

    ## Initializes a new Camera instance.
    # @param framerate The rate at which images are published.
    # @param queue The queue size of the image publishing.

    def __init__(self, framerate, queue):
        super().__init__('camera')
        self.stream = self.create_publisher(Image, 'camera_stream', queue)
        self.timer = self.create_timer(1/framerate, self.capture)
        self.camera = cv2.VideoCapture(4)
        #self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
        #self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cv2.imshow('camera', self.camera.read()[1])
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        self.bridge = CvBridge()

    ## Captures an image from a video device, sets its timestamp and publishes it.

    def capture(self):
        ret, original = self.camera.read()
        timestamp = time()

        if not ret:
            self.get_logger().error('Image capturing failed.')
            sleep(1)
            return

        msg = self.bridge.cv2_to_imgmsg(original)
        msg.header.stamp.sec = int(timestamp)
        msg.header.stamp.nanosec = int((timestamp-int(timestamp))*1000000000)
        self.stream.publish(msg)
        self.get_logger().info('Image published.')

def main(args=None):
    rclpy.init(args=args)
    camera = Camera(30, 3)
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()