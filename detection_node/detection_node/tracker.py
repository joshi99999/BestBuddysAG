import numpy as np
import math
import time
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ro45_portalrobot_interfaces.msg import IdSample, IdPosTime

from cv_bridge import CvBridge

## An EuclideanDistTracker instance is used to track and detected Objects, in an image.
#
#  To work properly the following conditions must be met.
#  + The given image must be a binary image.
#  + Objects need to be big enough.
#  + Objects need to be bright enough.
#  + Objects should not contact each other to detect an track them properly

class EuclideanDistTracker:

    ## Initializes a new EuclideanDistTracker instance.
    def __init__(self):
        # Store the center positions of the objects
        self.__center_points = {}

        self.__id_count = 0
        self.__id_prev = -1

        self.__cv_bridge = CvBridge()

    ## Updates the euclideandisttracker to track objects
    # @param objects_rect A list which contains the x and y coordinates and the width and heigth of the bounding boxes 
    # @return obcets_bbs_ids A list which contains the ids and the boundings boxes of the tracked objects 
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

            for id, pt in self.__center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 100:
                    self.__center_points[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True


            # NEW OBJECT DETECTION
            if same_object_detected is False:
                self.__center_points[self.__id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.__id_count])
                self.__id_count += 1
                

        # ASSIGN NEW ID to OBJECT
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.__center_points[object_id]
            new_center_points[object_id] = center

        self.__center_points = new_center_points.copy()
        return objects_bbs_ids


    ## Creates an image of an object if it is in a certain range
    # @param frame The image with objects of which samples should be taken
    # @param center_x The x-coordinate of the object center point
    # @param id The id of the object
    # @return publish A boolean value that is true if a sample was taken and false if not
    # @return id_img A ros2 msg which contains the id of the object and the taken sample of the object
    def getSample(self, frame, center_x, id):
        publish = False
        if (frame is not None):
            id_img = IdSample()
            num1 = Int32()

            minRange = 200
            maxRange = 250

            topBelt = 0
            bottomBelt = 175
            halfSampleResolution = 100


            # Object is in Range and is diffrent from previous object
            if (center_x >= minRange and center_x <= maxRange) and (id != self.__id_prev):
                # Create sample
                sample = frame[topBelt:bottomBelt, center_x-halfSampleResolution:center_x+halfSampleResolution]
                self.__id_prev = id

                #cv2.imwrite("src/Samples/Miau/Katze_sample_"+str(id+189)+".jpg", sample)

                # Set all the data for ros message
                num1.data = id
                id_img.id = num1
                ros_image = self.__cv_bridge.cv2_to_imgmsg(sample, encoding='8UC1')
                id_img.image = ros_image
                publish = True
                return publish, id_img
            
            else: 
                return publish, id_img
            
    
    ## Detects objects in a given image 
    # @param frame The input image 
    # @return detections A list which contains the x and y coordinates and the width and heigth of the bounding boxes 
    def detectObject(self, frame):
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
    
    ## Tracks objects by using update function
    # @param detections A list which contains the x and y coordinates and the width and heigth of the bounding boxes
    # @return id The id of the tracked object
    # @return cx The x-coordinate of the object center point
    # @return cy the y-coordinate of the object center point
    def trackObject(self, detections):
        if (detections is not None) and (len(detections)>0):
            boxes_ids = self.update(detections)
            for box_id in boxes_ids:
                x, y, w, h, id = box_id
                # Centerpoints
                cx = int((x + x + w)/2)
                cy = int((y + y + h)/2)
                return id, cx, cy
        else: return -1, 0, 0
    
    ## Calculates the center of mass of an image
    # @param image The image 
    # @param x The offset in x direction
    # @param y The offset in y direction
    def find_center_of_mass(self, image, x, y):
        # Find contours in image
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # if there are no contours return None
        if len(contours) == 0:
            return None

        # find largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # calculate momentum
        moments = cv2.moments(largest_contour)

        # calculate center of mass
        centroid_x = int(moments['m10'] / moments['m00'])
        centroid_y = int(moments['m01'] / moments['m00'])
        cx = centroid_x + x
        cy = centroid_y + y

        return cx, cy


