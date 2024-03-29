#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_msgs.msg import Int32, Int64

from ro45_portalrobot_interfaces.msg import IdSample, IdPosTime

from . import tracker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

## Tracker instance
tracker = tracker.EuclideanDistTracker()

## @package detection_node
# This package contains image detection tools.

## Converts parameters into ROS2 format.
# @param IdPosTime A ROS2 message that consists of id, x-position, y-position and current time.
# @param id The ID of an object.
# @param posx The x-position of an object.
# @param posy The y-position of an object.
# @param time The timestemp of the current image.
# @return IdPosTime The converted ROS2 message.
def convertToRos(IdPosTime, id, posx, posy, time):
    id_msg = Int32()
    position_x = Int32()
    position_y = Int32()
    time_msg = Int64()

    id_msg.data = id
    position_x.data = posx
    position_y.data = posy
    time_msg.data = int(time)

    IdPosTime.id = id_msg
    IdPosTime.pos_x = position_x
    IdPosTime.pos_y = position_y
    IdPosTime.time = time_msg

    return IdPosTime

## An ObjectDetection instance is used to create a ROS2 node for image detection.
# The node subscribes the topic "preprocessed_stream", 
# processes incoming images from that topic 
# and publishes the processed images on the topic "id_sample" for classification
# and publishes the processed images on the topic "id_pos_time" for modelling.
#
# The detection includes:
# + Detecting objects in the image.
# + Tracking the objects with their positions.
# + Creating samples for classification.
class ObjectDetection(Node):
    ## Initializes a new ObjectDetection instance.
    def __init__(self):
        super().__init__('detector')
        self.__image_subscriber = self.create_subscription(Image, 'preprocessed_stream', self.image_callback, 10)
        self.__id_sample_publisher = self.create_publisher(IdSample, 'id_sample', 10)
        self.__id_pos_publisher = self.create_publisher(IdPosTime, 'id_pos_time', 10)
        self.__cv_bridge = CvBridge()

    ## Callback function for processing image messages.
    # This function detects and tracks objects in the image.
    # It also creats samples for the classification.
    # @param msg The ROS2 message preprocessed_stream containing the binary image of the belt.
    def image_callback(self, msg):
        try:
            # Konvertiere das ROS-Bild zu einem OpenCV-Bild
            cv_image = self.__cv_bridge.imgmsg_to_cv2(msg, '8UC1')
        except CvBridgeError as e:
            self.get_logger().error('Fehler beim Konvertieren des ROS-Bildes: %s' % str(e))
            return
        
        time = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1000000


        # 1. Object detection
        detections = tracker.detectObject(cv_image)

        # 2. Object tracking
        boxes_ids = tracker.update(detections)
        for box_id in boxes_ids:
            x, y, w, h, id = box_id

            center_x = (x + x + w) // 2
            center_y = (y + y + h) // 2

            # Create image of Object
            image_object = cv_image[y:(y+h), x:(x+w)]
            # Find coordinates of center of mass from objekt
            cx, cy = tracker.find_center_of_mass(image_object, x, y)
            print("ID: "+str(id)+" Position: "+str(cx)+ " / "+str(cy))
            print("")
            print("")

            IdPos = IdPosTime()
            IdPos = convertToRos(IdPos, id, int(cx), int(cy), time)
            self.__id_pos_publisher.publish(IdPos)
            
            # 3. Create Image of Object
            publish, id_img = tracker.getSample(cv_image, center_x, center_y, id)
            if publish:
                #self.get_logger().info('publishing sample with id: '+str(id))
                print("sample with id: "+str(id))
                self.__id_sample_publisher.publish(id_img)
            # Draw center point
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
        
        cv2.imshow("bild", cv_image)
        cv2.waitKey(25)
    


def main(args=None):
    rclpy.init(args=args)
    objekt_detektion = ObjectDetection()
    rclpy.spin(objekt_detektion)
    objekt_detektion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
