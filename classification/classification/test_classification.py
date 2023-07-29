import cv2
import numpy as np
import unittest
from . import classification
import joblib
from joblib import dump , load
from sklearn import svm

model_file = 'src/classification/classification/svm_model.joblib'
svm_model = load(model_file)

class TestObjectClassification(unittest.TestCase):
    
    ##to test the function find_center_of_masse()
    def test_find_center_of_mass(self):
        # Test case 1: No objects (no contours detected)
        binary_image = np.zeros((100, 100), dtype=np.uint8)  # Blank image with all zeros
        center = find_center_of_mass(binary_image)
        self.assertIsNone(center)

        # Test case 2: One object in the center
        binary_image[50:60, 50:60] = 255  # White square in the center
        center = find_center_of_mass(binary_image)
        self.assertEqual(center, (54, 54))

        # Test case 3: Multiple objects, find center of the largest object
        binary_image[20:30, 20:30] = 255  # Another small white square
        center = find_center_of_mass(binary_image)
        self.assertEqual(center, (54, 54))  # The center of the larger object should still be (54, 54)
    
    ##to test the function find_gripping_point_vector()
    def test_find_gripping_point_vector(self):
        # Test case 1: Gripping point and center point coincide
        gripping_point = (50, 50)
        center_point = (50, 50)
        vector = find_gripping_point_vector(gripping_point, center_point)
        self.assertEqual(vector, (0, 0))  # The vector should be (0, 0) as the points coincide

        # Test case 2: Gripping point above the center
        gripping_point = (50, 40)
        center_point = (50, 50)
        vector = find_gripping_point_vector(gripping_point, center_point)
        self.assertEqual(vector, (0, -10))  # The vector should be (0, -10)

        # Test case 3: Gripping point to the right of the center
        gripping_point = (60, 50)
        center_point = (50, 50)
        vector = find_gripping_point_vector(gripping_point, center_point)
        self.assertEqual(vector, (10, 0))  # The vector should be (10, 0)
    
    ## function to test the classification 
    def test_object_classification(self):
        # Test case 1: Classify a cat image
        cat_image = cv2.imread('src/classification/classification/test_image_katze')  # Replace with the path to a unicorn image
        features, _ = feature_extract(cat)
        class_result = pred(features, svm_model)
        self.assertEqual(class_result, 0)  # Expected class 0 for a cat

        # Test case 2: Classify a unicorn image
        unicorn_image = cv2.imread('src/classification/classification/test_image_Einhorn')  # Replace with the path to a unicorn image
        features, _ = feature_extract(unicorn)
        class_result = pred(features, svm_model)
        self.assertEqual(class_result, 1)  # Expected class 1 for a unicorn

        # Test case 3: Classify an empty image (no objects)
        empty_image = np.zeros((100, 100), dtype=np.uint8)
        features, _ = feature_extract(empty_image)
        class_result = pred(features, svm_model)
        self.assertEqual(class_result, -1)  # Expected result -1 when no objects are found

if __name__ == '__main__':
    unittest.main()
