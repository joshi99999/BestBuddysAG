#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from portal_robot_interfaces.msg import IdClassVec
from portal_robot_interfaces.msg import IdPosVelTime
from portal_robot_interfaces.msg import PosVelClass


class SynchBlock(Node):
    def __init__(self):
        super().__init__('synchroniser')
        self.class_subscriber = self.create_subscription(IdClass, 'id_class_vec', self.class_callback, 10)
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
        cl = msg.classification.data
        # TODO: vector in message mit aufnehemn

        print("recieved object with id: "+str(id)+" and class: "+str(cl))

        if cl != -1:
            id_classification = [id, cl]
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
        vel = msg.vel_x
        time = msg.time

        print("recieved object with id: "+str(id)+" and position: "+str(pos_x)+" / "+str(pos_y)+" and speed: "+str(vel))

        for ids in reversed(self.ids_to_ignore):
            if ids == id:
                self.ids_to_ignore.remove(id)
                ignore = True
                print("ignored object with id: "+str(id))
                break
        if not ignore:
            for id_class in self.id_classes:
                if id_class[0] == id:
                    # remove id_class form list
                    # TODO: vector über id_class auf position addieren
                    print([id,id_class[0], pos_x, pos_y, vel, id_class[1]])
                    self.publish_synch(pos_x, pos_y, vel, id_class[1], time)
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
        num1 = Int32()
        num2 = Int32()
        num3 = float()
        num4 = Int32()
        num5 = Int32()


        num1.data = pos_x
        num2.data = pos_y
        num3 = vel
        num4.data = cl
        num5.data = time


        msg.pos_x = num1
        msg.pos_y = num2
        msg.vel_x = num3
        msg.classification = num4
        msg.time = num5


        print("published object with position: "+str(pos_x)+" / "+str(pos_y)+" speed: "+str(vel)+" and class: "+str(cl))

        self.pos_vel_class_publisher.publish(msg)
         

def main(args=None):
    rclpy.init(args=args)
    synch = SynchBlock()
    rclpy.spin(synch)
    synch.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


       