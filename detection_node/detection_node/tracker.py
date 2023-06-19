import numpy as np
import math
import time
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ro45_portalrobot_interfaces.msg import IdSample, IdPosTime

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

                if dist < 100:
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

    
    def getSample(self, frame, center_x, cy, id):
        """
        This function creates an slice of an image at a certain position if the x coordinate of an object is there

        Args:
            image (Numpy Array), x coordinate of object (int), id of object (int)
        Returns:
            image (Numpy Array), id of object (int)
        """
        publish = False
        if (frame is not None):
            id_img = IdSample()
            num1 = Int32()

            minRange = 320
            maxRange = 330

            topBelt = 0
            bottomBelt = 189
            halfSampleResolution = 100


            # Object is in Range and is diffrent from previous object
            if (center_x >= minRange and center_x <= maxRange) and (id != self.id_prev):
                # Create sample
                sample = frame[topBelt:bottomBelt, center_x-halfSampleResolution:center_x+halfSampleResolution]
                self.id_prev = id

                # Set all the data for ros message
                num1.data = id
                id_img.id = num1
                ros_image = self.cv_bridge.cv2_to_imgmsg(sample, encoding='8UC1')
                id_img.image = ros_image
                publish = True
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
            tuple: A tuple containing the ID of the tracked object, and its center coordinates (center_x, cy).

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
    

    def find_center_of_mass(self, image, x, y):
        # Finden der Konturen der Objekte im binären Bild
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Falls keine Konturen gefunden wurden, gibt es keine Objekte
        if len(contours) == 0:
            return None

        # Finden des größten Objekts basierend auf der Konturfläche
        largest_contour = max(contours, key=cv2.contourArea)

        # Berechnung der Momente des größten Objekts
        moments = cv2.moments(largest_contour)

        # Berechnung des Schwerpunkts (Centroid) des größten Objekts
        centroid_x = int(moments['m10'] / moments['m00'])
        centroid_y = int(moments['m01'] / moments['m00'])
        cx = centroid_x + x
        cy = centroid_y + y

        # Rückgabe des Schwerpunkts (Centroid)
        return cx, cy

