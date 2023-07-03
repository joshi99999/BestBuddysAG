#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import IdPosVelTime, IdPosTime
import math
from std_msgs.msg import Int32, Float32, Int64
import numpy as np
import cv2
    
class PositionGruppierung(Node):
    def __init__(self):
        """
        Initializes the PositionGrouping node.

        """
        super().__init__('PositionGruppierung')
        self.subscriber= self.create_subscription(IdPosTime, 'id_pos_time', self.function_callback, 10)
        self.publisher = self.create_publisher(IdPosVelTime, 'id_pos_vel_time', 10)
        self.list_for_element = []
            
    def function_callback(self, IdPosTime):
        """
        Callback function that is called when an IdPosTime message is received.

        parameters:
            IdPosTime: The received Id, Position  and Time.

        """
        id = IdPosTime.id.data
        pos_x = IdPosTime.pos_x.data
        pos_y = IdPosTime.pos_y.data
        timestamp = IdPosTime.time.data
        ret, target_id = self.addlist(id, timestamp, pos_x, pos_y)
        if ret:
            removed_elements=self.remove_elements_with_id(target_id)
            ret, id_value, pos_x_value, pos_y_value, speed_x_value, time_value = self.speed_determine(removed_elements) 
            if ret:
                print(speed_x_value)
                
                msg = IdPosVelTime()
                id_msg = Int32()
                id_msg.data = id_value

                pos_x_msg = Int32()
                pos_x_msg.data = pos_x_value

                pos_y_msg = Int32()
                pos_y_msg.data = pos_y_value

                time_msg = Int64()
                time_msg.data = time_value
                
                msg.id = id_msg
                msg.pos_x = pos_x_msg
                msg.pos_y = pos_y_msg
                msg.vel_x = speed_x_value
                msg.time = time_msg

                self.get_logger().info('Publishing: "%s"' % msg)
                self.publisher.publish(msg)

    #Alternate function to speed_determine
    def calculate_movement(list):
        id = list[-1][0]
        array = np.array(list, dtype=np.float32)
        y = np.average(array[:,3])        
        vx, vy, t, x = cv2.fitLine(array[:,1:3], distType=cv2.DIST_L2)
        v = vy/vx
        return True, id, x, y, v, t

    def speed_determine(self, list):
        """
        Calculates the speed based on position and time information in a list.

        Parameters:
            list (list): A list of elements, where each element has the following structure:
                        [ID, timestamp, position_X, position_Y]

        Returns:
            tuple: A tuple containing the following values:
                - ID of the last element in the list
                - X-position of the last element
                - Y-position of the last element
                - Speed in the X-direction
                - Timestamp of the last element
        """
        id = list[-1][0]

        start_pos_x = list[0][2]
        end_pos_x = list[-1][2]
        end_pos_y = list[-1][3]

        start_time = list[0][1]
        end_time = list[-1][1]

        if(len(list) < 2):
            return False, id, end_pos_x, end_pos_y, 0.0, end_time
        
        # pixel per second
        pixel_time = (end_pos_x - start_pos_x) / ((end_time - start_time)/1000)
        # 25 pixel are equal to 1cm
        scale = 25
        # m per second
        speed_x = (pixel_time / scale) / 100

        return True, id, end_pos_x, end_pos_y, speed_x, end_time 
    
    def remove_elements_with_id(self, target_id):
        """
        Removes all items with the specified ID from a list and returns them as a separate list.

        parameters:
            list_for_element (list): The list to remove elements from.
            target_id: The ID of the items to be removed.

        returns:
            list: A separate list containing the removed items.

        """
        removed_elements = [element for element in self.list_for_element if element[0] == target_id]
        self.list_for_element[:] = [element for element in self.list_for_element if element[0] != target_id]
        return removed_elements
        
    def addlist(self, id , timestamp, pos_x, pos_y):
        """
        Adds a new element to the list , sorts the list by ID and timestamp and Checks if the object is still in the tracking area.

        parameters:
            id: The ID of the new element.
            timestamp: The timestamp of the new element.
            pos_x: The x-position of the new element.
            pos_y: The y-position of the new element.

        returns:
            None
            
        """

        max_pos_x= 710

        if pos_x >= max_pos_x:
            return True, id
        
        self.list_for_element.append([id, timestamp, pos_x, pos_y])
            
def main(args=None):
    """
    Main function to start object classification.

    """
    rclpy.init(args=args)
    Position_Gruppierung = PositionGruppierung()
    rclpy.spin(Position_Gruppierung)
    Position_Gruppierung.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
