#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import PosVelClass, RobotPos, ConCmd, Error
from time import time

## @package logic
# This package contains logic modelling tools.

## A logic Controller to control the robot's movements based on received messages and data.
# The Logic Controller class represents the main logic for controlling the robot's movements.
# The node is responsible for controling the robot's behavior, which includes fetching and picking up objects from a conveyor belt,
# moving to specific positions, and handling error states. It subscribes to several ROS2 messages to receive information about the robot's
# state, the velocity of the conveyor belt, and the current position of the robot. It also publishes ROS2 messages to command the robot's movements.
# The logic includes:
# + Moving the robot to the initialize position after start.
# + Moving the robot to the desired positions.
# + Sorting the objects in the boxes.

class Controller(Node):

    ## Initializes a new Controller instance.    
    # @param __box_cat_x X position of the box for the cat.
    # @param box_unicorn_x X position of the box for the unicorn.
    # @param box_y Y position of the box, which is the same for both the unicorn and the cat.
    # @param wait Array with three values for the waiting position of the robot, which is central over the conveyor belt.
    # @param pickup_z Z position for the object pickup position from the conveyor belt.
    # @param y_enabled_z Z height to enable the y-axis movement if the z value is equal to or higher than this height.
    # @param error Variable to check if an error occurred.
    # @param time Variable to save the time of the object callback.
    # @param type Variable to save the type of the object callback. For a cat, the value is 0, and for a unicorn, the value is 1.
    # @param state Variable to save the current state and robot movement of the controller.
    # @param msg_out ROS2 message to publish the desired robot position.
    # @param init_time Variable to save the time of the init state.
    # @param init_duration Variable to save the duration of the init state. This is set to 10 seconds, allowing enough time for every motor to drive to the init position.
    # @param position Array to store the robot's current x and y positions.
    # @param reference Array to store the current reference positions for x, y, and z axes.
    # @param error_subscriber ROS2 subscription for the error state messages from the Error topic.
    # @param object_subscriber ROS2 subscription for the object information (position, velocity, and type) from the PosVelClass topic.
    # @param position_subscriber ROS2 subscription for the current position of the robot from the RobotPos topic.
    # @param publisher ROS2 publisher for the calculated velocity to the RobotCmd topic.
  
    def __init__(self):
        super().__init__('logic')

        self.__box_cat_x = 0.0
        self.__box_unicorn_x = 0.1
        self.__box_y = 0.1
        self.__wait = [0.1, 0.02, 0.06]
        self.__pickup_z = 0.074
        self.__y_enabled_z = 0.04

        self.__error_subscriber = self.create_subscription(Error, 'error', self.error_callback, 10)
        self.__object_subscriber = self.create_subscription(PosVelClass, 'pos_vel_class', self.object_callback, 10)
        self.__position_subscriber = self.create_subscription(RobotPos, 'robot_position', self.robotPosition_callback, 10)
        self.__publisher = self.create_publisher(ConCmd, 'controller_command', 10)    

        self.__error = False

        self.__msg_out = ConCmd()
        self.__msg_out.vel_x, self.__msg_out.grip = 0.0, False

        self.__time = 0.0
        self.__type = None    #id: 0= Katze, 1 = Einhorn
        self.__state = 'init'

        self.__init_time = None
        self.__init_duration = 10
        self.__position = np.zeros(2, dtype=np.float64)
        self.__reference = np.zeros(3, dtype=np.float64)

    ## Error callback function to get the error message and set the error variable to true if the function gets called.
    # This function gets called when an error message gets called and sets the error value to True.
    # With the ROS2 logger the error message gets displayed on the terminal.
    # @param msg The error message.
    @staticmethod
    def error_callback(self, msg):
        self.__error = True
        self.get_logger().error('An error occured: "%s"' % msg)
        if self.__state != 'init' and self.__state != 'reference_z':
            self.__state = 'error_z'
        
    ## Object callback function to get the object message and set the velocity in x direction to the negative value of the object velocity.
    # @param time Variable to save the time of the object callback.
    # @param msg The object message.
    @staticmethod
    def object_callback(self, msg):
        if 0 < self.__msg_out.vel_x or self.__error or self.__state == 'init' or self.__state == 'reference_z':
            return
        self.__msg_out.vel_x = -msg.vel_x
        self.__time = msg.time.data / 1000
        self.__type = msg.result.data
        self.__position[:] = self.__reference[0] + msg.pos_x, self.__reference[1] + msg.pos_y
    
    ## Robot position callback function to get the robot position message and set the robot position in dependence of the current state.
    # @param msg_in The robot position message.
    #
    ## Cases: 
    # The robot moves to different positions in dependence of the current state.
    ## + Case 'init' to move the step motor from the z-axis to the initialize position.
    # The case publishes an updated new robot position that moves the robot for 10 seconds straight to the minimum border of the z-axis.
    # After the 10 seconds, the z position is 0.0, and the next case 'reference_z' is called.
    #
    ## + Case 'reference_z' to move the step motors from the x and y-axis to the initialize position.
    # The case publishes an updated new robot position that moves the robot for 10 seconds straight to the minimum borders of the x and y-axis.
    # After the 10 seconds, the x and y positions are set to 0.0, and a new robot position is published with the defined __wait position, which gets called after that.
    #
    ## + Case 'wait' to move the robot to the defined robot waiting position.
    # Since the wait position is already being published in the case 'reference_z', the robot moves without a new publish to the wait position.
    # The robot waits until a new object is detected, and the vel_x is unequal to zero.
    # After a new object is detected, the y and z values of the robot get updated, and the next case 'fetch_x' gets called.  
    #
    ## + Case 'fetch_x' to move the robot along the x-axis to a target position.
    # In the 'fetch_x' case, the robot is controlled to move along the x-axis to reach the object position.
    # The robot trys to chatch the object on the conveyor belt by moving along the x-axis with the object velocity.
    # If the robot can't catch the object, the robot moves to the wait position.
    # If the robot catches the object, the robot moves to the Case 'lifting'.
    #
    ## + Case 'lifting' to pick up the reached object and move the z-axis up.
    # In the 'lifting' case, the robot picks up the reached object by moving the z-axis upwards until the value is greater than or equal to the wait position.
    # After reaching the desired z position, the x position gets updated to the x position of the unicorn or cat box, depending on the lifted object type.
    # The z position gets updated to the value where the y-axis is free to move, and the updated x and z positions get published.
    # After publishing the new positions, the case 'y_enabling' is called.
    #
    ## + Case 'y_enabling' moves the y position of the robot to the y position of the boxes.
    # The y position gets updated after the z position reaches the position where y can move without any danger of crashing or losing the object.
    # After that, the y and z positions are updated and published to the y and z positions of the object box.
    # After the publish, the next case 'xy_moving' is called.
    #
    ## + Case 'xy_moving' moves the robot over the object box and releases the vacuum to sort the object in the box.
    # After the x and y positions are close enough to the desired position, the gripper releases the vacuum, and the current position is published with the updated gripper value.
    # The x and y positions are updated and published to the wait position.
    # The next case 'y_disabling' is called after the position is published.
    #
    ## + Case y_disabling moves the z position to the wait position.
    # After the robot is over the desired x and y position the z position is updated and published.
    # The next case 'wait' gets called after publishing the new z positon.
    #
    ## + Case 'error_z'
    # If there is an error, the gripper set to False and the z axis moves to the z reference position by publishing the updated robot position.
    # After publishing the case 'error_xy' gets called.
    #
    ## + Case 'error_xy' moves the x and y axis to the reference position.
    # After the z axis reaches the reference position the x and y position gets updated and the robot position is published.
    # The next error case 'error' gets called after publishing.
    #
    ## + Case 'error' checks if the x and y position is reached and calls the next case 'dead'
    #
    ## + Casse 'dead' 
    # The placeholder pass gets called and after that the programm is in a dead state. 
    @staticmethod
    def robotPosition_callback(self, msg_in):
        self.get_logger().info(self.__state)

        match self.__state:

            # initialize the robots z axis
            case 'init':
                if self.__init_time is None:
                    self.__msg_out.pos_x, self.__msg_out.pos_y = msg_in.pos_x, msg_in.pos_y
                    self.__init_time = time()
                
                if time() - self.__init_time < self.__init_duration:
                    self.__msg_out.pos_z = msg_in.pos_z - 0.01
                    self.__publisher.publish(self.__msg_out)
                ## \brief Continuously moves the robot towards the minimum z-axis border until the init_duration is reached.    
                else:
                    self.__init_time = None
                    self.__reference[2] = msg_in.pos_z
                    self.__wait[2] += msg_in.pos_z
                    self.__pickup_z += msg_in.pos_z
                    self.__y_enabled_z += msg_in.pos_z
                    self.__state = 'reference_z'

            # initialize the robots x and y axis
            case 'reference_z':
                ## \brief Initializes the case 'reference_z' and sets the z-position.
                if self.__init_time is None:
                    self.__msg_out.pos_z = msg_in.pos_z
                    self.__init_time = time()
                ## \brief Continuously moves the robot towards the minimum x and y-axis borders until the init_duration is reached.    
                if time() - self.__init_time < self.__init_duration:
                    self.__msg_out.pos_x, self.__msg_out.pos_y = msg_in.pos_x - 0.01, msg_in.pos_y - 0.01
                    self.__publisher.publish(self.__msg_out)
                else:
                    self.__reference[0:2] = msg_in.pos_x, msg_in.pos_y
                    self.__box_cat_x += msg_in.pos_x
                    self.__box_unicorn_x += msg_in.pos_x
                    self.__box_y += msg_in.pos_y
                    self.__wait[0] += msg_in.pos_x
                    self.__wait[1] += msg_in.pos_y
                    self.__msg_out.pos_x, self.__msg_out.pos_y, self.__msg_out.pos_z = self.__wait
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'dead' if self.__error else 'wait'

            # move to the wait position
            case 'wait':
                if 0 != self.__msg_out.vel_x:
                    self.__msg_out.pos_y, self.__msg_out.pos_z = self.__position[1], self.__wait[2]
                    self.get_logger().info(str(self.__position[1]))
                    self.__state = 'fetch_x'
            # try to chatch the detected object
            case 'fetch_x':
                self.__msg_out.pos_x = self.__position[0] + (time() - self.__time) * self.__msg_out.vel_x
                if self.__msg_out.pos_x < self.__reference[0]:
                    self.__msg_out.pos_x, self.__msg_out.pos_y, self.__msg_out.pos_z = self.__wait[0], self.__wait[1], self.__wait[2]
                    self.__msg_out.vel_x = 0.0
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'wait'
                elif np.linalg.norm([msg_in.pos_x - self.__msg_out.pos_x, msg_in.pos_y - self.__msg_out.pos_y]) < 0.004 and abs(msg_in.pos_z - self.__msg_out.pos_z) < 0.001:   
                    if self.__msg_out.pos_z < self.__pickup_z:
                        self.__msg_out.pos_z, self.__msg_out.pos_y = self.__pickup_z, msg_in.pos_y
                    else:
                        self.__msg_out.grip, self.__msg_out.vel_x, self.__msg_out.pos_x, self.__msg_out.pos_z = True, 0.0, msg_in.pos_x, self.__wait[2]
                        self.__state = 'lifting'
                self.__publisher.publish(self.__msg_out)

 
            # lift the chatched object 
            case 'lifting':
                ## \brief Check if the robot's z position is close enough to the wait position.
                if abs(msg_in.pos_z - self.__wait[2]) < 0.001:
                    self.__msg_out.pos_x = self.__box_cat_x if self.__type == 0 else self.__box_unicorn_x
                    self.__msg_out.pos_z = self.__y_enabled_z
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'y_enabling'

           # move to the y position of the box
            case 'y_enabling':
                ## \brief Check if the robot's z position is close enough to the y-enabled z position.
                if abs(msg_in.pos_z - self.__y_enabled_z) < 0.001:
                    self.__msg_out.pos_y, self.__msg_out.pos_z = self.__box_y, msg_in.pos_z
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'xy_moving'

            # move over the box from the detected and chatched object and release the vacuum
            case 'xy_moving':
                # (Add implementation details here, if applicable)
                if np.linalg.norm([msg_in.pos_x - self.__msg_out.pos_x, msg_in.pos_y - self.__msg_out.pos_y]) < 0.001:
                    self.__msg_out.grip, self.__msg_out.pos_x, self.__msg_out.pos_y = False, msg_in.pos_x, msg_in.pos_y
                    self.__publisher.publish(self.__msg_out)
                    self.__msg_out.pos_x, self.__msg_out.pos_y = self.__wait[0], self.__wait[1]
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'y_disabling'
            
            # move to the wait position
            case 'y_disabling':
                if abs(msg_in.pos_y - self.__wait[1]) < 0.001:
                    self.__msg_out.pos_z = self.__wait[2]
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'wait'
            
            # error state
            case 'error_z':
                self.__msg_out.pos_x, self.__msg_out.pos_y, self.__msg_out.pos_z = msg_in.pos_x, msg_in.pos_y, self.__reference[2]
                self.__msg_out.vel_x, self.__msg_out.grip = 0.0, False
                self.__publisher.publish(self.__msg_out)
                self.__state = 'error_xy'
            
            
            case 'error_xy':
                if abs(self.__reference[2] - msg_in.pos_z) < 0.001:
                    self.__msg_out.pos_x, self.__msg_out.pos_y = self.__reference[0:2]
                    self.__msg_out.pos_z = msg_in.pos_z
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'error'
            
            case 'error':
                if np.linalg.norm([self.__reference[0] - msg_in.pos_x, self.__reference[1] - msg_in.pos_y]) < 0.001:
                    self.__msg_out.pos_x, self.__msg_out.pos_y = self.__reference[0], self.__reference[1]
                    self.__publisher.publish(self.__msg_out)
                    self.__state = 'dead'

           
            case 'dead':
                pass
                
## Main function to create and run the Position Controller node.
def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
