import unittest
from tracker import*
import cv2
import numpy as np

class TestDetection(unittest.TestCase):

    def setUp(self):
        self.tracker = EuclideanDistTracker()
    
    # Test function detectObject()
    def test_no_object(self): # OK
        # Teste mit einem Bild, das kein Objekt enthält
        no_objects = np.zeros((175, 800),  dtype=np.uint8)
        detections = self.tracker.detectObject(no_objects)
        self.assertEqual(detections, [])
    
    def test_object_left(self): # OK
        # Teste mit einem Bild, das ein Objekt am linken Rand enthält
        no_objects = np.zeros((175, 800),  dtype=np.uint8)
        one_object_left_side = cv2.rectangle(no_objects, (0, 50), (0 + 50, 50 + 100), (255, 255, 255), -1)
        detections = self.tracker.detectObject(one_object_left_side)
        self.assertEqual(len(detections), 1)
        self.assertListEqual(detections[0], [0, 50, 51, 101])

    def test_object_middle(self): # OK
        # Teste mit einem Bild, das ein Objekt in der Mitte enthält
        no_objects = np.zeros((175, 800),  dtype=np.uint8)
        one_object_middle = cv2.rectangle(no_objects, (400, 50), (400 + 100, 50 + 100), (255, 255, 255), -1)
        detections = self.tracker.detectObject(one_object_middle)
        self.assertEqual(len(detections), 1)
        self.assertListEqual(detections[0], [400, 50, 101, 101])

    def test_two_objects(self): # OK
        # Teste mit einem Bild, das zwei Objekte enthält
        no_objects = np.zeros((175, 800),  dtype=np.uint8)
        one_object = cv2.rectangle(no_objects, (100, 50), (100 + 100, 50 + 100), (255, 255, 255), -1)
        second_object = cv2.rectangle(one_object, (600, 50), (600 + 100, 50 + 100), (255, 255, 255), -1)
        detections = self.tracker.detectObject(second_object)
        self.assertEqual(len(detections), 2)
        self.assertListEqual(detections[0], [600, 50, 101, 101])
        self.assertListEqual(detections[1], [100, 50, 101, 101])

    def test_object_right(self): # OK
        # Teste mit einem Bild, das ein Objekt am rechten Rand enthält
        no_objects = np.zeros((175, 800),  dtype=np.uint8)
        one_object_right_side = cv2.rectangle(no_objects, (750, 50), (750 + 50, 50 + 100), (255, 255, 255), -1)
        detections = self.tracker.detectObject(one_object_right_side)
        self.assertEqual(len(detections), 1)
        self.assertListEqual(detections[0], [750, 50, 50, 101])
    
    # Test tracking
    def test_tracking_one_object(self):
        no_objects = np.zeros((175, 800),  dtype=np.uint8)
        i = 0
        while(i<200):
            one_object = cv2.rectangle(no_objects, (10+i, 50), (10+i + 100, 50 + 100), (255, 255, 255), -1)
            detections = self.tracker.detectObject(one_object)
            id, cx, cy = self.tracker.trackObject(detections)
            result = [id, cx, cy]
            one_object = no_objects
            i += 2
        self.assertEqual(result, [0, 159, 100])

    def test_tracking_no_object(self):
        no_objects = np.zeros((175, 800),  dtype=np.uint8)
        detections = self.tracker.detectObject(no_objects)
        id, cx, cy = self.tracker.trackObject(detections)
        result = [id, cx, cy]
        self.assertEqual(result, [-1, 0, 0])
 

if __name__ == '__main__':
    unittest.main()