import numpy as np
import math
import time
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from portal_robot_interfaces.msg import IdSample
from portal_robot_interfaces.msg import IdPosTime

from cv_bridge import CvBridge

class EuclideanDistTracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}

        self.id_count = 0
        self.id_prev = -1

        self.cv_bridge = CvBridge()

    def update(self, objects_rect):
        """
        Updates the object tracking based on the given object rectangles.

        Args:
            objects_rect (list): A list of rectangles (x, y, w, h) representing the detected objects.

        Returns:
            list: A list of object bounding boxes (x, y, w, h, id) that have been updated.

        """

        objects_bbs_ids = []

        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            # CHECK IF OBJECT IS DETECTED ALREADY
            same_object_detected = False

            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 70:
                    self.center_points[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True


            # NEW OBJECT DETECTION
            if same_object_detected is False:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1
                

        # ASSIGN NEW ID to OBJECT
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        self.center_points = new_center_points.copy()
        return objects_bbs_ids

    
    def getSample(self, frame, cx, id):
        """
        This function creates an slice of an image at a certain position if the x coordinate of an object is there

        Args:
            image (Numpy Array), x coordinate of object (int), id of object (int)
        Returns:
            image (Numpy Array), id of object (int)
        """
        if (frame is not None) and (len(frame)>0):
            id_img = IdSample()
            num = Int32()
            publish = False
            if (cx>=320 and cx<=325) and (id != self.id_prev):
                # Create sample
                sample = frame[0:300, cx-100:cx+100]
                self.id_prev = id

                # Set all the data for message
                num.data = id
                id_img.id = num
                ros_image = self.cv_bridge.cv2_to_imgmsg(sample, encoding='8UC1')
                id_img.image = ros_image
                return publish, id_img
            
            else: 
                return publish, id_img
            
    
    def detectObject(self, frame):
        """
        Detects objects in the given frame.

        Args:
            frame: The input frame to detect objects from.

        Returns:
            list: A list of detected object rectangles (x, y, w, h).

        """
        detections = []
        contours, _=cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            # Calculate area and remove small elements
            area = cv2.contourArea(cnt)
            if area > 1000:
                # Draw Rectangle bounding box
                x, y, w, h = cv2.boundingRect(cnt)
                detections.append([x, y, w, h])
        return detections
    
    
    def trackObject(self, detections):
        """
        Tracks an object based on the given detections.

        Args:
            detections (list): A list of detected object rectangles (x, y, w, h).

        Returns:
            tuple: A tuple containing the ID of the tracked object, and its center coordinates (cx, cy).

        """
        if (detections is not None) and (len(detections)>0):
            boxes_ids = self.update(detections)
            for box_id in boxes_ids:
                x, y, w, h, id = box_id
                # Centerpoints
                cx = int((x + x + w)/2)
                cy = int((y + y + h)/2)
                return id, cx, cy
        else: return -1, 0, 0
