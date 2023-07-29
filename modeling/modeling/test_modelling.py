import cv2
import numpy as np
import unittest
from . import modelling

## class to test the modeller
class TestModeler(unittest.TestCase):

    # testcase: check if calculate_movement returns correct movement data
    def test_calculate_movement(self):
        point_list = [[100000, 500, 200], [200000, 700, 300], [300000, 900, 400]]
        result = calculate_movement(point_list)
        expected_result = (0.28, 0.1200, 0.00079, 200000)

         # Use assertAlmostEqual to compare the floating-point values with a tolerance of 0.0001
        self.assertAlmostEqual(result[0], expected_result[0], places=4)
        self.assertAlmostEqual(result[1], expected_result[1])
        self.assertAlmostEqual(result[2], expected_result[2], places=4)
        self.assertAlmostEqual(result[3], expected_result[3])
    
    # Testcase: check if calculate_movement return None if less than two points are passed
    def test_calculate_movement_invalid_input(self):
        point_list = [[100000, 500, 200]]
        result = calculate_movement(point_list)
        self.assertIsNone(result)


    # Testcase: check if get_positions_by_id throws a ValueError if the id is not in the list
    def test_get_positions_by_id_invalid_id(self):
        id_list = [1, 2, 3]
        position_lists = [[[100000, 500, 200]], [[200000, 700, 300] ,[300000, 900, 400]], [[400000, 600, 500]]]
        id_list = id_list
        position_lists = position_lists
        with self.assertRaises(ValueError):
            get_positions_by_id(4)

    # Testcase: check if get_positions_by_id returns the correct positions for a valid id
    def test_get_positions_by_id_valid_id(self):
        id_list = [1, 2, 3]
        position_lists = [[[100000, 500, 200]], [[200000, 700, 300]], [[300000, 900, 400]]]
        id_list = id_list  
        position_lists = position_lists  
        
        result = get_positions_by_id(1)

        expected_positions = [[100000, 500, 200]]
        self.assertEqual(result, expected_positions)


    # Testcase: check if add_position a element with invalid x-Position not added to the Liste 
    def test_add_position_invalid_x_position(self):
        add_position(1, 100000, 800, 200)  # valid x-Position: 0 bis 709
        self.assertEqual(id_list, [])
        self.assertEqual(position_lists, [])

    # Testcase: check if add_position one element with valid x-position added to the list
    def test_add_position_valid_x_position(self):
        add_position(1, 100000, 500, 200)  # valid x-Position: 0 bis 709
        self.assertEqual(id_list, [1])
        self.assertEqual(position_lists, [[[100000, 500, 200]]])
    

if __name__ == '__main__':
    unittest.main()