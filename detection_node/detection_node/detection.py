#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_msgs.msg import Int32

from ro45_portalrobot_interfaces.msg import IdSample, IdPosTime

from . import tracker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Create tracker object
tracker = tracker.EuclideanDistTracker()

def convertToRos(IdPosTime, id, posx, posy, time):
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
    num4 = Int32()

    num1.data = id
    num2.data = posx
    num3.data = posy
    num4.data = time

    IdPosTime.id = num1
    IdPosTime.pos_x = num2
    IdPosTime.pos_y = num3
    IdPosTime.time = num4

    return IdPosTime

class ObjektDetektion(Node):
    def __init__(self):
        super().__init__('detector')
        self.image_subscriber = self.create_subscription(Image, 'frame', self.image_callback, 10)
        self.id_sample_publisher = self.create_publisher(IdSample, 'id_sample', 10)
        self.id_pos_publisher = self.create_publisher(IdPosTime, 'IdPosTime', 10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Konvertiere das ROS-Bild zu einem OpenCV-Bild
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, '8UC1')
        except CvBridgeError as e:
            self.get_logger().error('Fehler beim Konvertieren des ROS-Bildes: %s' % str(e))
            return
        
        time = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1000000
        #print(time)

        # 1. Object detection
        detections = tracker.detectObject(cv_image)

        # 2. Object tracking
        boxes_ids = tracker.update(detections)
        for box_id in boxes_ids:
            x, y, w, h, id = box_id

            # Create image of Object
            image_object = cv_image[y:(y+h), x:(x+w)]
            # Find coordinates of center of mass from objekt
            cx, cy = tracker.find_center_of_mass(image_object, x, y)
            print("center"+str(cx)+ " / "+str(cy)+" with id: "+str(id))
            
            # Draw center point
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

            IdPos = IdPosTime()
            IdPos = convertToRos(IdPos, id, int(cx), int(cy), time)
            self.id_pos_publisher.publish(IdPos)
            
            # 3. Create Image of Object
            publish, id_img = tracker.getSample(cv_image, cx, id, time)
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
