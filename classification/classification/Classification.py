#!/usr/bin/env python3
import cv2
import numpy as np
import pandas as pd
from sklearn import svm
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ro45_portalrobot_interfaces.msg import IdSample, IdClassVec
from std_msgs.msg import Int32 
import joblib

class ObjectClassification(Node):
    """
    Class for object classification.

    Attributes:
        subscription: ROS subscription for the input image.
        publisher: ROS publisher for the classification result.
        bridge: OpenCV bridge for converting ROS images.
        svm_model: SVM model for classification.

    """

    def __init__(self):
        """
        Initializes the object classification.

        """
        super().__init__('ObjectClassification')
        self.subscription = self.create_subscription(IdSample, 'id_sample', self.image_callback, 10)
        self.publisher = self.create_publisher(IdClassVec, 'id_class_vec', 10)
        self.bridge = CvBridge()

        model_path = 'src/classification/classification/svm_model.pkl'
        self.svm_model = joblib.load(model_path)
        

    def image_callback(self, IdSample):
        """
        Callback function for the input image.

        Parameters:
            Image: ROS image.
        """
        cv_image = self.bridge.imgmsg_to_cv2(IdSample.image, desired_encoding='8UC1')
        features ,gripping_point , gravity = feature_extract(cv_image)
        vector = find_gripping_point_vector(gripping_point, gravity)
        class_result = pred(features, self.svm_model)
        print(class_result[0])
        print(IdSample.id.data)
        msg = IdClassVec()
        
        id_msg = Int32()
        id_msg.data = IdSample.id.data
        
        class_result_msg = Int32()
        class_result_msg.data = int(class_result)
        
        vector_x_msg = Int32()
        vector_x_msg.data = int(vector[0])
        
        vector_y_msg = Int32()
        vector_y_msg.data = int(vector[1])
        
        msg.id = id_msg
        msg.result = class_result_msg
        msg.vector_x = vector_x_msg
        msg.vector_y = vector_y_msg 

        self.get_logger().info('Publishing: "%s"' % msg)
        
        self.publisher.publish(msg)


def main(args=None):
    """
    Main function to start object classification
    
    """
    rclpy.init(args=args)
    object_classification = ObjectClassification()
    rclpy.spin(object_classification)
    object_classification.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
def find_gripping_point_vector(gripping_point, center_point):

    vector = (gripping_point[0] - center_point[0], gripping_point[1] - center_point[1])

    return vector

    
def center_of_gravity(img):
    """
    Calculates the centroid of a binary image.

    Parameters:
        img (ndarray): Binary image.

    Returns:
        np.ndarray: Centroid coordinates (y, x).

    """
    try:
        yind, xind = np.where(img)
        return np.array((np.mean(yind), np.mean(xind)))
    except Exception as e:
        print("Fehler beim Berechnen des Schwerpunkts:", str(e))
        return None


def inertia_tensor(binary_img):
    """
    Calculates the inertia tensor of a binary image.

    Parameters:
        binary_img (ndarray): Binary image.

    Returns:
        np.ndarray: Inertia tensor matrix.

    """
    try:
        gravity_y, gravity_x = center_of_gravity(binary_img)
        matrix_inertia_tensor = np.zeros((2, 2))
        y, x = np.where(binary_img)
        y = y - gravity_y
        x = x - gravity_x
        number_of_pixel = float(len(y))
        matrix_inertia_tensor[0, 0] = np.sum(y**2) / number_of_pixel
        matrix_inertia_tensor[1, 1] = np.sum(x**2) / number_of_pixel
        matrix_inertia_tensor[0, 1] = -np.sum(x*y) / number_of_pixel
        matrix_inertia_tensor[1, 0] = matrix_inertia_tensor[0, 1]
        return matrix_inertia_tensor
    except Exception as e:
        print("Fehler beim Berechnen des Trägheitstensors:", str(e))
        return None
  
   
def Hauptmoment(e_vals):
    """
    Calculates the principal moments based on the eigenvalues.

    Parameters:
        e_vals (ndarray): Eigenvalues.

    Returns:
        I_x (float): Principal moment along the x-axis.
        I_y (float): Principal moment along the y-axis.

    """
    try:
        major_eigenvalue_idx = np.argmax(e_vals)
        minor_eigenvalue_idx = np.argmin(e_vals)
        major_eigenvalue = e_vals[major_eigenvalue_idx]
        minor_eigenvalue = e_vals[minor_eigenvalue_idx]
        I_x = major_eigenvalue 
        I_y = minor_eigenvalue 
        return I_x, I_y
    except TypeError:
        print("Invalid input. The 'e_vals' parameter must be a valid ndarray.")
        return None, None

def feature_extract(image):
    """
    Classifies an image as a cat or unicorn.

    Parameters:
        image (str): Path to the image.

    Returns:
        features (list): List of extracted features of the image.
        gripping_point : A tuple containing the gripping point as (x, y) coordinates.
        gravity : List containing the gravity_x and the gravity_y
        
    """
    try:
        brightest_pixel = np.unravel_index(np.argmax(image), image.shape)
        gripping_point = (brightest_pixel[1], brightest_pixel[0])
        blur = cv2.GaussianBlur(image, (5, 5), 0)
        edges = cv2.Canny(blur, 300, 525)
        num_edges = cv2.countNonZero(edges)
        dst = cv2.cornerHarris(image, 2, 3, 0.04)
        threshold = 0.02 * dst.max()
        corners = []
        for i in range(dst.shape[0]):
            for j in range(dst.shape[1]):
                if dst[i, j] > threshold:
                    corners.append((i, j))
        gravity_y, gravity_x = center_of_gravity(image)
        gravity =[gravity_x, gravity_y]
        cv2.circle(edges, (int(gravity_x + 0.5), int(gravity_y + 0.5)), 5, (0, 0, 255), -1)
        e_vals, _ = np.linalg.eig(inertia_tensor(image))
        try:
            I_x, I_y = Hauptmoment(e_vals)
        except TypeError:
            print("Der e_vals-Parameter ist kein gültiges ndarray")
        cv2.imshow('Prediction', edges)
        cv2.waitKey(1)
        features = [num_edges, I_x, I_y, len(corners)]
        return features , gripping_point, gravity
    except cv2.error as e:
        print("Fehler beim Lesen des Bildes:", str(e))
        return None


def pred(features, model):
    """
    Performs a prediction using the SVM model and prints the result.

    Parameters:
        features (list): List of image features.
        model: SVM model.

    Returns:
        int: Prediction class (0 for cat, 1 for unicorn).

    """
    try:
        prediction = model.predict([features])
        if( prediction !=0 and prediction !=1):
            prediction = -1
        return prediction
    except ValueError as e:
        print("Fehler bei der Vorhersage:", str(e))
