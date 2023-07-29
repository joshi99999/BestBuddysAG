#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import IdPosVelTime, IdPosTime
import numpy as np
import cv2

## @package modeling This package contains movement modeling tools.

## A Modeler instance is used to create a ROS2 node for movement modeling.

class Modeler(Node):
    
    ##Initializes the PositionGrouping node.
    def __init__(self):
        super().__init__('modeler')

        self.old_ids = []        
        self.id_list = []
        self.position_lists = []


        self.subscriber= self.create_subscription(IdPosTime, 'id_pos_time', self.position_callback, 10)
        self.publisher = self.create_publisher(IdPosVelTime, 'id_pos_vel_time', 10)
    
    ##This callback function is called automatically when an IdPosTime message is received. It processes the received message, adding the position and time data to the internal storage. If the Id is new, it calculates the movement from stored positions and publishes the result.
    # @param  msg_in (IdPosTime): The received IdPosTime message containing the Id, Position, and Time data.
    # @return none
    def position_callback(self, msg_in):
        try:
            if msg_in.id.data in self.old_ids:
                return

            id = self.add_position(msg_in.id.data, msg_in.time.data, msg_in.pos_x.data, msg_in.pos_y.data)

            if id is not None:
                positions = self.get_positions_by_id(id)
                ret = Modeler.calculate_movement(positions)
                if ret is not None:
                    msg_out = IdPosVelTime()
                    msg_out.id = id
                    msg_out.pos_x, msg_out.pos_y, msg_out.vel_x, msg_out.time = ret
                    self.get_logger().info('Publishing: "%s"' % msg_out)
                    self.publisher.publish(msg_out)
        except Exception as e:
            # Catch any unexpected exceptions that may occur within the method
            print(f"An unexpected error occurred in position_callback: {e}")

    ##Calculates movement data from a list of points.
    # @param list (List): A list of points, where each point is represented as a 3-tuple (x, y, t). Here, x denotes the horizontal position, y denotes the vertical position, and t denotes time.
    # @return Tuple: A 4-tuple containing the calculated movement data: (shifted_horizontal_position, vertical_position, velocity, shifted_time)
    @staticmethod 
    def calculate_movement(list):
        try:
            if len(list) < 2:
                raise ValueError("At least two points are required to calculate movement.")
            array = np.array(list, dtype=np.float64)
            y = np.average(array[:,2]).astype(np.float64) 
            offset = int(min(array[:,0])/100000)*100000
            array[:,0] = array[:,0] - offset
            vx, vy, t, x = cv2.fitLine(array[:,0:2], distType=cv2.DIST_L2, param = 0, reps = 0.01, aeps = 0.01).flatten().astype(np.float64)
            v = vy/vx
            return x*0.0004, y*0.0004, v*0.4, int(t+offset)
        except ValueError as e :
            print(str(e))
            return None
        except:
            # Catch any other unexpected exceptions that may occur
            return None
    
    ##Removes all items with the specified ID from a list and returns them as a separate list.
    # @param  list_for_element (list): The list to remove elements from.
    # @param id: The ID of the items to be removed.
    # @return list: A separate list containing the removed items.
    def get_positions_by_id(self, id):
        try:
            index = self.id_list.index(id)
            positions = self.position_lists.pop(index)
            self.id_list.remove(id)
            self.old_ids.append(id)
            return positions
        except ValueError as e :
            # Catch the ValueError if the specified ID is not found in the id_list
            raise ValueError("ID not found in the list:", str(e))
        
    ##Adds a newelement to the list , sorts the list by ID and timestamp and Checks if the object is still in the tracking area.
    # @param id: The ID of the new element.
    # @param timestamp: The timestamp of the new element.
    # @param pos_x: The x-position of the new element.
    # @param pos_y: The y-position of the new element.
    # @return None
    def add_position(self, id , timestamp, pos_x, pos_y):
        max_pos_x = 710
        
        try:
            # Check if the x-position is within the tracking area
            if pos_x >= max_pos_x:
                raise ValueError("Invalid position: x-position is outside the tracking area.")

            # Validate other parameters if needed
            # For example, you might want to check if the provided timestamp, pos_x, or pos_y are valid.

            if id not in self.id_list:
                self.id_list.append(id)
                self.position_lists.append([[timestamp, pos_x, pos_y]])
            else:
                self.position_lists[self.id_list.index(id)].append([timestamp, pos_x, pos_y])

        except ValueError as e:
            # Catch the ValueError and handle it within the method
            print("Error: ", str(e))
            
##Main function to start object classification.
def main(args=None):
    rclpy.init(args=args)
    modeler = Modeler()
    rclpy.spin(modeler)
    modeler.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
