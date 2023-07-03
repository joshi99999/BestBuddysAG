#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int64
from ro45_portalrobot_interfaces.msg import IdClassVec, IdPosVelTime, PosVelClass

class SynchBlock(Node):
    def __init__(self):
        super().__init__('synchroniser')
        self.class_subscriber = self.create_subscription(IdClassVec, 'id_class_vec', self.class_callback, 10)
        self.pos_subscriber = self.create_subscription(IdPosVelTime, 'id_pos_vel_time', self.pos_callback, 10)
        self.pos_vel_class_publisher = self.create_publisher(PosVelClass, 'pos_vel_class', 10)

        self.id_classes = []
        self.ids_to_ignore = []

    
    def class_callback(self, msg):
        """
        Callback function for processing classification messages.

        Args:
            msg (IdClass): Classification message containing object ID and class.

        Returns:
            None
        """
        id = msg.id.data
        cl = msg.result.data
        vector_x = msg.vector_x.data
        vector_y = msg.vector_y.data
        
        print("RECIEVED object with id: "+str(id)+" and class: "+str(cl))

        if cl != -1:
            id_classification = [id, cl, vector_x, vector_y]
            self.id_classes.append(id_classification)
        else:
            self.ids_to_ignore.append(id)
    
    def pos_callback(self, msg):
        """
        This function checks if the message can be ignoered because its neither a cat or unicorn 
        if this is true it compares the id of the message with the id of the class-message 
        if they are the same it publishes the new message

        Args:
            msg (IdPosVel): Position message containing object ID, position, and velocity.

        Returns:
            None
        """
        ignore = False

        id = msg.id.data
        pos_x = msg.pos_x.data
        pos_y = msg.pos_y.data
        velocity = msg.vel_x
        time = msg.time.data

        print("RECIEVED object with id: "+str(id)+" and position: "+str(pos_x)+" / "+str(pos_y)+" and speed: "+str(velocity))

        for ids in reversed(self.ids_to_ignore):
            if ids == id:
                self.ids_to_ignore.remove(id)
                ignore = True
                #print("ignored object with id: "+str(id))
                break
        if not ignore:
            for id_class in self.id_classes:
                if id_class[0] == id:

                    vector_x = id_class[2]
                    vector_y = id_class[3]
                    classification = id_class[1]
                    #pos_x, pos_y = self.calculatePoint(pos_x, pos_y, vector_x, vector_y)
                    
                    self.publish_synch(pos_x, pos_y, velocity, classification, time)

                    self.id_classes.remove(id_class)


    def publish_synch(self, pos_x, pos_y, vel, cl, time):
        """
        This function creates and publishes a PosVelClass message with the 4 given integers after converting them to int32

        Args:
            pos_x (int): X-position of the object.
            pos_y (int): Y-position of the object.
            vel (float): Velocity of the object.
            cl (int): Classification of the object.

        Returns:
            None
        """
        # change to ros type
        msg = PosVelClass()
        position_x = float()
        position_y = float()
        velocity = float()
        classification = Int32()
        timestemp = Int64()

        # Change from pixle position to meter
        position_x = ((pos_x / 25)/100)
        position_y = ((pos_y / 25)/100)
        # Change to relativ position from robot
        position_x, position_y = self.calculateActualCoordinates(position_x, position_y)
        
        velocity = vel
        classification.data = cl
        timestemp.data = time


        msg.pos_x = position_x
        msg.pos_y = position_y
        msg.vel_x = velocity
        msg.result = classification
        msg.time = timestemp


        self.get_logger().info('Publishing: "%s"' % msg)

        self.pos_vel_class_publisher.publish(msg)
         
    def calculatePoint(self, x1, y1, vector_x, vector_y):

        new_x = x1 + vector_x
        new_y = y1 + vector_y

        return new_x, new_y

    def calculateActualCoordinates(self, x, y):
        scale_image_width = 0.32
        scale_image_height = 0.07
        gap = 0.1
        robot_x_axis = 0.10
        robot_y_axis = 0.015

        # distance from scale_image (32cm) + robot x-axis (10cm) + gap (10cm)
        distance_x = scale_image_width + robot_x_axis + gap
        # distance from scale_image_y (7cm) - robot_y zero_position (1,5 cm)
        distance_y = scale_image_height - robot_y_axis

        actual_pos_x = distance_x - x
        actual_pos_y = distance_y - y

        return actual_pos_x, actual_pos_y

def main(args=None):
    rclpy.init(args=args)
    synch = SynchBlock()
    rclpy.spin(synch)
    synch.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


       