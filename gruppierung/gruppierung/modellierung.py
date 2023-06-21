#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import IdPosVelTime, IdPosTime
import math
from std_msgs.msg import Int32, Float32, Int64
    
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
        if(len(list) < 2):
            return False, list[-1][0], list[-1][2], list[-1][3], 0.0, list[-1][1]
        #start_time = list[0][1]
        #end_time = list[-1][1]
        start_time = 1
        end_time = 2
        start_pos_x = list[0][2]
        end_pos_x = list[-1][2]
        speed_x = float((end_pos_x - start_pos_x) / (end_time - start_time))
        return True, list[-1][0], list[-1][2], list[-1][3], speed_x, list[-1][1] 
    
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
        new_element = [id, timestamp, pos_x, pos_y]
        max_pos_x= 1100
        
        isFull= False
        target_id = None
        self.list_for_element.append(new_element)
        self.list_for_element.sort(key=lambda x: (x[0], x[1]))
        
        for i, element in enumerate(self.list_for_element):
            # Checks if the object is still in the tracking area
            if i < len(self.list_for_element) - 1 and element[0] != self.list_for_element[i + 1][0]:
                if element[2] >= max_pos_x:
                    isFull = True
                    target_id = element[0]
        return isFull , target_id
            
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
