#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from portal_robot_interfaces.msg import IdSample
from portal_robot_interfaces.msg import IdPosTime

from . import tracker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Create tracker object
tracker = tracker.EuclideanDistTracker()

def convertToRos(IdPosTime, id, posx, posy):
    """
    Converts ID, position x, and position y values to a ROS message format.

    Args:
        IdPosTime: The ROS message object to populate.
        id (int): The ID value.
        posx (int): The x-position value.
        posy (int): The y-position value.

    Returns:
        IdPosTime: The populated ROS message object.

    """
    num1 = Int32()
    num2 = Int32()
    num3 = Int32()

    num1.data = id
    num2.data = posx
    num3.data = posy

    IdPosTime.id = num1
    IdPosTime.pos_x = num2
    IdPosTime.pos_y = num3

    return IdPosTime

class ObjektDetektion(Node):
    def __init__(self):
        super().__init__('detector')
        self.image_subscriber = self.create_subscription(Image, 'frames', self.bild_callback, 10)
        self.id_sample_publisher = self.create_publisher(IdSample, 'id_sample', 10)
        self.id_pos_publisher = self.create_publisher(IdPosTime, 'id_pos', 10)
        self.cv_bridge = CvBridge()

    def bild_callback(self, msg):
        try:
            # Konvertiere das ROS-Bild zu einem OpenCV-Bild
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, '8UC1')
        except CvBridgeError as e:
            self.get_logger().error('Fehler beim Konvertieren des ROS-Bildes: %s' % str(e))
            return
        
        # 1. Object detection
        detections = tracker.detectObject(cv_image)

        # 2. Object tracking
        boxes_ids = tracker.update(detections)
        for box_id in boxes_ids:
            x, y, w, h, id = box_id
            # Centerpoints
            cx = int((x + x + w)/2)
            cy = int((y + y + h)/2)
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

            IdPos = IdPosTime()
            IdPos = convertToRos(IdPos, id, cx, cy)
            self.id_pos_publisher.publish(IdPos)

            # 3. Create Image of Object
            publish, id_img = tracker.getSample(cv_image, cx, id)
            if publish:
                self.id_sample_publisher.publish(id_img)
        
        cv2.imshow("bild", cv_image)
        cv2.waitKey(1)
    


def main(args=None):
    rclpy.init(args=args)
    objekt_detektion = ObjektDetektion()
    rclpy.spin(objekt_detektion)
    objekt_detektion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
