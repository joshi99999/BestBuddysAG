#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import PosVelClass, RobotPos, ConCmd, Error
from time import time

## \file
# \brief A logic controller to control the robot's movements based on received messages and data.
#
# The Controller class is responsible for controling the robot's behavior, which includes fetching and picking up objects from a conveyor belt,
# moving to specific positions, and handling error states. It subscribes to several ROS2 messages to receive information about the robot's
# state, the velocity of the conveyor belt, and the current position of the robot. It also publishes ROS2 messages to command the robot's movements.
#
# \class Controller
# \brief Initializes a new Logic Controller instance.
# The Logic Controller class represents the main logic for controlling the robot's movements. It is initialized with specific parameters
# that define the positions of boxes, waiting positions, pickup height, and axis movement enabling thresholds.
# The class subscribes to ROS2 messages for error states, object information, and the robot's position.
# It publishes a ROS2 message to the controller with the desired robot positions.
#
# \param box_cat_x X position of the box for the cat.
# \param box_unicorn_x X position of the box for the unicorn.
# \param box_y Y position of the box, which is the same for both the unicorn and the cat.
# \param wait Array with three values for the waiting position of the robot, which is central over the conveyor belt.
# \param pickup_z Z position for the object pickup position from the conveyor belt.
# \param y_enabled_z Z height to enable the y-axis movement if the z value is equal to or higher than this height.
# \param error Variable to check if an error occurred.
# \param time Variable to save the time of the object callback.
# \param type Variable to save the type of the object callback. For a cat, the value is 0, and for a unicorn, the value is 1.
# \param state Variable to save the current state and robot movement of the controller.
# \param init_time Variable to save the time of the init state.
# \param init_duration Variable to save the duration of the init state. This is set to 10 seconds, allowing enough time for every motor to drive to the init position.
# \param position Array to store the robot's current x and y positions.
# \param reference Array to store the current reference positions for x, y, and z axes.
#
# Subscribes to the ROS2 message topics:
#   - 'error' : Error state messages
#   - 'pos_vel_class' : Object information (position, velocity, and type)
#   - 'robot_position' : Current robot position
#
# Publishes to the ROS2 message topic:
#   - 'controller_command' : Desired robot position commands.
#

class Controller(Node):

    ## \brief Initializes a new Logic Controller instance.
    #
    # The constructor sets up the Controller instance with default values and initializes the ROS2 communication.
    # It subscribes to the named topics and creates a publisher to send the robot position to the controller file.
    #
    # \param self The current instance of the class.
    # \return None
  
    def __init__(self):
        super().__init__('logic')

        self.box_cat_x = 0.0
        self.box_unicorn_x = 0.1
        self.box_y = 0.1
        self.wait = [0.1, 0.02, 0.06]
        self.pickup_z = 0.074
        self.y_enabled_z = 0.04

        self.error_subscriber = self.create_subscription(Error, 'error', self.error_callback, 10)
        self.object_subscriber = self.create_subscription(PosVelClass, 'pos_vel_class', self.object_callback, 10)
        self.position_subscriber = self.create_subscription(RobotPos, 'robot_position', self.robotPosition_callback, 10)
        self.publisher = self.create_publisher(ConCmd, 'controller_command', 10)    

        self.error = False

        self.msg_out = ConCmd()
        self.msg_out.vel_x, self.msg_out.grip = 0.0, False

        self.time = 0.0
        self.type = None    #id: 0= Katze, 1 = Einhorn
        self.state = 'init'

        self.init_time = None
        self.init_duration = 10
        self.position = np.zeros(2, dtype=np.float64)
        self.reference = np.zeros(3, dtype=np.float64)

    ## Error callback function to get the error message and set the error variable to true if the function gets called.
    #
    # This function gets called when an error message gets called and sets the error value to True.
    # With the ROS2 logger the error message gets displayed on the terminal.
    @staticmethod
    def error_callback(self, msg):
        self.error = True
        self.get_logger().error('An error occured: "%s"' % msg)
        if self.state != 'init' and self.state != 'reference_z':
            self.state = 'error_z'
        
    ## Object callback function to get the object message and set the velocity in x direction to the negative value of the object velocity.
    # @param time Variable to save the time of the object callback.
    @staticmethod
    def object_callback(self, msg):
        if 0 < self.msg_out.vel_x or self.error or self.state == 'init' or self.state == 'reference_z':
            return
        self.msg_out.vel_x = -msg.vel_x
        self.time = msg.time.data / 1000
        self.type = msg.result.data
        self.position[:] = self.reference[0] + msg.pos_x, self.reference[1] + msg.pos_y
    
    ## Robot position callback function to get the robot position message and set the robot position in dependence of the current state.
    @staticmethod
    def robotPosition_callback(self, msg_in):
        self.get_logger().info(self.state)

        match self.state:

            ## \brief Case 'init' to move the step motor from the z-axis to the initialize position.
            #
            # The case publishes an updated new robot position that moves the robot for 10 seconds straight to the minimum border of the z-axis.
            # After the 10 seconds, the z position is 0.0, and the next case 'reference_z' is called.
            case 'init':
                if self.init_time is None:
                    self.msg_out.pos_x, self.msg_out.pos_y = msg_in.pos_x, msg_in.pos_y
                    self.init_time = time()
                
                if time() - self.init_time < self.init_duration:
                    self.msg_out.pos_z = msg_in.pos_z - 0.01
                    self.publisher.publish(self.msg_out)
                ## \brief Continuously moves the robot towards the minimum z-axis border until the init_duration is reached.    
                else:
                    self.init_time = None
                    self.reference[2] = msg_in.pos_z
                    self.wait[2] += msg_in.pos_z
                    self.pickup_z += msg_in.pos_z
                    self.y_enabled_z += msg_in.pos_z
                    self.state = 'reference_z'

            ## \brief Case 'reference_z' to move the step motors from the x and y-axis to the initialize position.
            #
            # The case publishes an updated new robot position that moves the robot for 10 seconds straight to the minimum borders of the x and y-axis.
            # After the 10 seconds, the x and y positions are set to 0.0, and a new robot position is published with the defined wait position, which gets called after that.
            case 'reference_z':
                ## \brief Initializes the case 'reference_z' and sets the z-position.
                if self.init_time is None:
                    self.msg_out.pos_z = msg_in.pos_z
                    self.init_time = time()
                ## \brief Continuously moves the robot towards the minimum x and y-axis borders until the init_duration is reached.    
                if time() - self.init_time < self.init_duration:
                    self.msg_out.pos_x, self.msg_out.pos_y = msg_in.pos_x - 0.01, msg_in.pos_y - 0.01
                    self.publisher.publish(self.msg_out)
                else:
                    self.reference[0:2] = msg_in.pos_x, msg_in.pos_y
                    self.box_cat_x += msg_in.pos_x
                    self.box_unicorn_x += msg_in.pos_x
                    self.box_y += msg_in.pos_y
                    self.wait[0] += msg_in.pos_x
                    self.wait[1] += msg_in.pos_y
                    self.msg_out.pos_x, self.msg_out.pos_y, self.msg_out.pos_z = self.wait
                    self.publisher.publish(self.msg_out)
                    self.state = 'dead' if self.error else 'wait'

            ## \brief Case 'wait' to move the robot to the defined robot waiting position.
            #
            # Since the wait position is already being published in the case 'reference_z', the robot moves without a new publish to the wait position.
            # The robot waits until a new object is detected, and the vel_x is unequal to zero.
            # After a new object is detected, the y and z values of the robot get updated, and the next case 'fetch_x' gets called.
            case 'wait':
                if 0 != self.msg_out.vel_x:
                    self.msg_out.pos_y, self.msg_out.pos_z = self.position[1], self.wait[2]
                    self.get_logger().info(str(self.position[1]))
                    self.state = 'fetch_x'

            ## \brief Case 'fetch_x' to move the robot along the x-axis to a target position.
            #
            # In the 'fetch_x' case, the robot is controlled to move along the x-axis to reach a target position.
            # The robot's position is calculated based on its initial position, velocity, and time elapsed since the movement started.
            # If the robot reaches the target x position defined in 'reference[0]', it stops its movement, and the state changes to 'wait'.
            # If the robot's x and y position is close enough to the target position 'msg_in.pos_x' and 'msg_in.pos_y',
            # and the z position is within a tolerance of '0.001' of 'msg_in.pos_z', the robot prepares for lifting by adjusting its z position.
            # If the current z position 'self.msg_out.pos_z' is less than 'self.pickup_z', the robot's z position is set to 'self.pickup_z'
            # while keeping its y position unchanged to align with the target object's position.
            # Otherwise, the robot's grip is enabled, and its x position and z position are set to 'msg_in.pos_x' and 'self.wait[2]', respectively,
            # preparing the robot for the 'lifting' state.
            #
            # \param self The current instance of the class.
            # \param msg_in The input message containing information about the robot's current position.
            # \return None
            case 'fetch_x':
                self.msg_out.pos_x = self.position[0] + (time() - self.time) * self.msg_out.vel_x
                if self.msg_out.pos_x < self.reference[0]:
                    self.msg_out.pos_x, self.msg_out.pos_y, self.msg_out.pos_z = self.wait[0], self.wait[1], self.wait[2]
                    self.msg_out.vel_x = 0.0
                    self.publisher.publish(self.msg_out)
                    self.state = 'wait'
                elif np.linalg.norm([msg_in.pos_x - self.msg_out.pos_x, msg_in.pos_y - self.msg_out.pos_y]) < 0.004 and abs(msg_in.pos_z - self.msg_out.pos_z) < 0.001:   
                    if self.msg_out.pos_z < self.pickup_z:
                        self.msg_out.pos_z, self.msg_out.pos_y = self.pickup_z, msg_in.pos_y
                    else:
                        self.msg_out.grip, self.msg_out.vel_x, self.msg_out.pos_x, self.msg_out.pos_z = True, 0.0, msg_in.pos_x, self.wait[2]
                        self.state = 'lifting'
                self.publisher.publish(self.msg_out)

 
            ## \brief Case 'lifting' to pick up the reached object and move the z-axis up.
            #
            # In the 'lifting' case, the robot picks up the reached object by moving the z-axis upwards until the value is greater than or equal to the wait position.
            # After reaching the desired z position, the x position gets updated to the x position of the unicorn or cat box, depending on the lifted object type.
            # The z position gets updated to the value where the y-axis is free to move, and the updated x and z positions get published.
            # After publishing the new positions, the case 'y_enabling' is called.
            case 'lifting':
                ## \brief Check if the robot's z position is close enough to the wait position.
                if abs(msg_in.pos_z - self.wait[2]) < 0.001:
                    self.msg_out.pos_x = self.box_cat_x if self.type == 0 else self.box_unicorn_x
                    self.msg_out.pos_z = self.y_enabled_z
                    self.publisher.publish(self.msg_out)
                    self.state = 'y_enabling'

            ## \brief Case 'y_enabling' moves the y position of the robot to the y position of the boxes.
            #
            # The y position gets updated after the z position reaches the position where y can move without any danger of crashing or losing the object.
            # After that, the y and z positions are updated and published to the y and z positions of the object box.
            # After the publish, the next case 'xy_moving' is called.
            case 'y_enabling':
                ## \brief Check if the robot's z position is close enough to the y-enabled z position.
                if abs(msg_in.pos_z - self.y_enabled_z) < 0.001:
                    self.msg_out.pos_y, self.msg_out.pos_z = self.box_y, msg_in.pos_z
                    self.publisher.publish(self.msg_out)
                    self.state = 'xy_moving'

            ## \brief Case 'xy_moving' moves the robot over the object box and releases the vacuum to sort the object in the box.
            #
            # After the x and y positions are close enough to the desired position, the gripper releases the vacuum, and the current position is published with the updated gripper value.
            # The x and y positions are updated and published to the wait position.
            # The next case 'y_disabling' is called after the position is published.
            case 'xy_moving':
                # (Add implementation details here, if applicable)
                if np.linalg.norm([msg_in.pos_x - self.msg_out.pos_x, msg_in.pos_y - self.msg_out.pos_y]) < 0.001:
                    self.msg_out.grip, self.msg_out.pos_x, self.msg_out.pos_y = False, msg_in.pos_x, msg_in.pos_y
                    self.publisher.publish(self.msg_out)
                    self.msg_out.pos_x, self.msg_out.pos_y = self.wait[0], self.wait[1]
                    self.publisher.publish(self.msg_out)
                    self.state = 'y_disabling'
            ## \brief Case y_disabling moves the z position to the wait position.
            #
            # After the robot is over the desired x and y position the z position is updated and published.
            # The next case 'wait' gets called after publishing the new z positon.
            case 'y_disabling':
                if abs(msg_in.pos_y - self.wait[1]) < 0.001:
                    self.msg_out.pos_z = self.wait[2]
                    self.publisher.publish(self.msg_out)
                    self.state = 'wait'
            ## \brief Case 'error_z'
            # 
            # If there is an error, the gripper set to False and the z axis moves to the z reference position by publishing the updated robot position.
            # After publishing the case 'error_xy' gets called.
            case 'error_z':
                self.msg_out.pos_x, self.msg_out.pos_y, self.msg_out.pos_z = msg_in.pos_x, msg_in.pos_y, self.reference[2]
                self.msg_out.vel_x, self.msg_out.grip = 0.0, False
                self.publisher.publish(self.msg_out)
                self.state = 'error_xy'
            ## \brief Case 'error_xy' moves the x and y axis to the reference position.
            #
            # After the z axis reaches the reference position the x and y position gets updated and the robot position is published.
            # The next error case 'error' gets called after publishing.
            case 'error_xy':
                if abs(self.reference[2] - msg_in.pos_z) < 0.001:
                    self.msg_out.pos_x, self.msg_out.pos_y = self.reference[0:2]
                    self.msg_out.pos_z = msg_in.pos_z
                    self.publisher.publish(self.msg_out)
                    self.state = 'error'
            ## \brief Case 'error' checks if the x and y position is reached and calls the next case 'dead'
            case 'error':
                if np.linalg.norm([self.reference[0] - msg_in.pos_x, self.reference[1] - msg_in.pos_y]) < 0.001:
                    self.msg_out.pos_x, self.msg_out.pos_y = self.reference[0], self.reference[1]
                    self.publisher.publish(self.msg_out)
                    self.state = 'dead'

            ## \brief Casse 'dead' 
            # 
            # The placeholder pass gets called and after that the programm is in a dead state. 
            case 'dead':
                pass
                
## \file
# \brief Main function to create and run the Position Controller node.
#  
def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
