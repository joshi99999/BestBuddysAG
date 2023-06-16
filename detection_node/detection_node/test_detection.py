import unittest
import tracker
import cv2

from std_msgs.msg import Int32
from portal_robot_interfaces.msg import IdClass, IdPosVel, IdPosTime

class TestDetection(unittest.TestCase):

    def setUp(self) -> None:
        return super().setUp()

    def test_detectObject(self):

        image = cv2.imread()

        detections = tracker.detectObject(image)
        
