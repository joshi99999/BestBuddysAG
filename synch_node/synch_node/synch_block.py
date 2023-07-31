#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int64
from ro45_portalrobot_interfaces.msg import IdClassVec, IdPosVelTime, PosVelClass

## @package synch_node.
# This package contains synchronising tools.

## A SynchBlock instance is used to create a ROS2 node for synchronisation.
# The node subscribes the topics "id_class_vec" from the classification
# and "id_pos_vel_time" from the modelling.
# It processes incoming data from those topics
# and publishes the synchronised message on the topic "pos_vel_class" for the logic.
#
# The synchronisation includes:
# + Data from the classification.
# + Data from the modelling.
class SynchBlock(Node):
    ## Initializes a new SynchBlock instance.
    def __init__(self):
        super().__init__('synchroniser')
        self.__class_subscriber = self.create_subscription(IdClassVec, 'id_class_vec', self.class_callback, 10)
        self.__pos_subscriber = self.create_subscription(IdPosVelTime, 'id_pos_vel_time', self.pos_callback, 10)
        self.__pos_vel_class_publisher = self.create_publisher(PosVelClass, 'pos_vel_class', 10)

        self.__id_classes = []
        self.__ids_to_ignore = []

    ## Callback function for processing classification messages.
    # @param msg The ROS2 message IdClass containing the ID and the classification of an object.
    def class_callback(self, msg):
        id = msg.id.data
        cl = msg.result.data
        vector_x = msg.vector_x.data
        vector_y = msg.vector_y.data
        
        print("RECIEVED object with id: "+str(id)+" and class: "+str(cl))

        if cl != -1:
            id_classification = [id, cl, vector_x, vector_y]
            self.__id_classes.append(id_classification)
        else:
            self.__ids_to_ignore.append(id)
    ## Callback function for processing modelling messages.
    # This function checks if the message can be ignoered because its neither a cat or unicorn 
    # if this is true it compares the id of the message with the id of the class-message, 
    # if they are the same it publishes the new message.
    # @param msg The ROS2 message IdPosVel containing the ID, position and the velocity of an object.
    def pos_callback(self, msg):
        ignore = False

        id = msg.id
        pos_x = msg.pos_x
        pos_y = msg.pos_y
        velocity = msg.vel_x
        time = msg.time

        print("RECIEVED object with id: "+str(id)+" and position: "+str(pos_x)+" / "+str(pos_y)+" and speed: "+str(velocity))

        for ids in reversed(self.__ids_to_ignore):
            if ids == id:
                self.__ids_to_ignore.remove(id)
                ignore = True
                #print("ignored object with id: "+str(id))
                break
        if not ignore:
            for id_class in self.__id_classes:
                if id_class[0] == id:

                    vector_x = id_class[2]*0.0004
                    vector_y = id_class[3]*0.0004
                    classification = id_class[1]
                    pos_x, pos_y = self.calculatePoint(pos_x, pos_y, vector_x, vector_y)
                    
                    self.publish_synch(pos_x, pos_y, velocity, classification, time)

                    self.__id_classes.remove(id_class)

    ## This function creates and publishes a PosVelClass message with the 4 given integers after converting them to int32.
    # @param pos_x The x-position of an object.
    # @param pos_y The y-position of an object.
    # @param vel The velocity of an object.
    # @param cl The classification of an object.
    # @param time The timestemp of the current object.
    def publish_synch(self, pos_x, pos_y, vel, cl, time):
        # change to ros type
        msg = PosVelClass()
        position_x = float()
        position_y = float()
        velocity = float()
        classification = Int32()
        timestemp = Int64()

        # Change to relativ position from robot
        position_x, position_y = self.calculateActualCoordinates(pos_x, pos_y)
        
        velocity = vel
        classification.data = cl
        timestemp.data = time


        msg.pos_x = position_x
        msg.pos_y = position_y
        msg.vel_x = velocity
        msg.result = classification
        msg.time = timestemp


        self.get_logger().info('Publishing: "%s"' % msg)

        self.__pos_vel_class_publisher.publish(msg)
         
    ## Calculating the gripping point based on the given vector.
    # @param x1 The x-positon of the center of mass.
    # @param y1 The y-positon of the center of mass.
    # @param vector_x The fist parameter of the vector.
    # @param vector_y The second parameter of the vector.
    # @return new_x The x-postion of the gripping point.
    # @return new_y The y-postion of the gripping point.
    def calculatePoint(self, x1, y1, vector_x, vector_y):

        new_x = x1 + vector_x
        new_y = y1 + vector_y

        return new_x, new_y
    ## Calculates the actual position of the object relativ to the gripper.
    # @param x The x-position of the object in the image.
    # @param y The y-position of the object in the image.
    # @return actual_pos_x The calculated x_position of the object relativ to the gripper.
    # @return actual_pos_y The calculated y_position of the object relativ to the gripper.
    def calculateActualCoordinates(self, x, y):
        distance_x = 0.507
        distance_y = 0.06   

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


       