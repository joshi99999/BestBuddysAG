#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import IdPosVelTime, IdPosTime
import numpy as np
import cv2
    
class Modeler(Node):
    def __init__(self):
        """
        Initializes the PositionGrouping node.

        """
        super().__init__('modeler')

        self.id_list = []
        self.position_lists = []

        self.subscriber= self.create_subscription(IdPosTime, 'id_pos_time', self.position_callback, 10)
        self.publisher = self.create_publisher(IdPosVelTime, 'id_pos_vel_time', 10)
            
    def position_callback(self, msg_in):
        """
        Callback function that is called when an IdPosTime message is received.

        parameters:
            IdPosTime: The received Id, Position  and Time.

        """
        id = self.add_position(msg_in.id.data, msg_in.time.data, msg_in.pos_x.data, msg_in.pos_y.data)
        if id is not None:
            positions = self.get_positions_by_id(id)
            ret = self.calculate_movement(positions)
            if ret is not None:
                msg_out = IdPosVelTime()
                msg_out.id = id
                msg_out.pos_x, msg_out.pos_y, msg_out.vel_x, msg_out.time = ret
                self.get_logger().info('Publishing: "%s"' % msg_out)
                self.publisher.publish(msg_out)

    #Alternate function to speed_determine
    def calculate_movement(list):
        if len(list) < 2:
            return
        array = np.array(list, dtype=np.float32)
        y = np.average(array[:,2])        
        vx, vy, t, x = cv2.fitLine(array[:,0:2], distType=cv2.DIST_L2)
        scale = 0.0004
        v = vy/vx*scale
        return x, y, v, t

    '''
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
    '''
    
    def get_positions_by_id(self, id):
        """
        Removes all items with the specified ID from a list and returns them as a separate list.

        parameters:
            list_for_element (list): The list to remove elements from.
            id: The ID of the items to be removed.

        returns:
            list: A separate list containing the removed items.

        """

        positions = self.position_lists.pop(self.id_list.index(id))
        self.id_list.remove(id)
        return positions
        
    def add_position(self, id , timestamp, pos_x, pos_y):
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
            return id
        
        if id not in self.id_list:
            self.id_list.append(id)
            self.position_lists.append([[timestamp, pos_x, pos_y]])
        else:
            self.position_lists[self.id_list.index(id)].append([timestamp, pos_x, pos_y])
            
def main(args=None):
    """
    Main function to start object classification.

    """
    rclpy.init(args=args)
    modeler = Modeler()
    rclpy.spin(modeler)
    modeler.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
