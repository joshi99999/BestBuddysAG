#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd, ConCmd
from time import time
## @package position_controller
# This package contains robot controller tools.

## A PositionController instance is used to create a ROS2 node for robot position control. 
# The node subscribes the topic "robot_position" and "controller_command", 
# calculates the new velocity for each step motor in dependence of the current position and the desired position
# and publishes the calculated velocity on the topic "robot_command".
# 
# The pos_control includes:
# + Calculation of the new velocity for each step motor in dependence of the current position and the desired position.
# + Regulate the velocity to the maximum velocity.
# + Regulate the velocity jump to the maximum acceleration.
# + Publish the calculated velocity for each step motor and the grip value as a RobotCmd message.

class PositionController(Node):

    ## Initializes a new PositionController instance.     
    #  @param k Proportional gain value for the velocity calculation.
    #  @param max_vel Array with three values for the maximum velocity of each step motor.
    #  @param max_acc Array with thee values for the maximum velocity of each step motor.
    #  @param desired_pos Desired position for each step motor.
    #  @param current_pos Current position for each step motor.
    #  @param velocity Current velocity for each step motor.
    #  @param vel_x_offset Offset for the x velocity.
    #  @param grip Boolean value to control the robot gripper.
    # @param subscriptionCurrent ROS2 subscription for the current position of the robot from the RobotPos topic.
    # @param subscriptionDesired ROS2 subscription for the desired position of the robot from the ConCmd topic.
    # @param publisher ROS2 publisher for the calculated velocity to the RobotCmd topic.
    def __init__(self):
        super().__init__('control_roboter_position')
        
        self.__k = np.array([3, 3, 3], dtype=np.float64)

        self.__max_vel = np.array([0.03, 0.04, 0.05], dtype=np.float64)
        self.__max_acc = np.array([0.005, 0.005, 0.02], dtype=np.float64)

        self.__desired_pos = np.zeros(3, dtype=np.float64)
        self.__current_pos = np.zeros(3, dtype=np.float64)
        self.__velocity = np.zeros(3, dtype=np.float64)
        
        self.__vel_x_offset = 0.0
        self.__grip = False

        # Subscribs the ROS2 message of the current position of the robot from the RobotPos and the desired position from the ConCmd topic.
        # Publishes the ROS2 message of the calculated velocity to the RobotCmd topic.
        self.__subscriptionCurrent = self.create_subscription(RobotPos, 'robot_position', self.robotPostion_callback, 10)
        self.__subscriptionDesired = self.create_subscription(ConCmd, 'controller_command', self.controllerCommand_callback, 10)

        self.__publisher = self.create_publisher(RobotCmd, 'robot_command', 10)
    

    ## Callback function to get the desired X, Y and Z position and the offset for the x velocity.
    # The grip value is set to the desired grip value.
    #  @param msg The ConCmd message containing the desired position and velocity offset.
    @staticmethod
    def controllerCommand_callback(self, msg):
        self.__desired_pos[:] = [msg.pos_x, msg.pos_y, msg.pos_z]
        self.__vel_x_offset, self.__grip = msg.vel_x, msg.__grip
    
    ## Callback function to get the current position X, Y and Z and print the velocity in the consol.
    #  @param msg The RobotPos message containing the current position.
    #  @return robot_command returns the calculated velocity for each step motor and the grip value as a RobotCmd message.
    @staticmethod
    def robotPostion_callback(self, msg):
        self.__current_pos[:] = msg.pos_x, msg.pos_y, msg.pos_z

        cmd = self.calculate_control_signal()
        self.__publisher.publish(cmd)

        self.get_logger().info('Publishing velocities: ' + str(cmd))

    ## Calculates the new velocity for each step motor in dependence of the current position and the desired position.
    # For the velocity in x direction the offset has to be added because with this the system is in rest position.
    # The velocity is set to the maximum velocity when the velocity is higher than the maximum velocity.
    # The velocity is set to the maximum acceleration when the velocity jump is higher than the maximum acceleration.
    # @return robot_command Returns the calculated velocity for each step motor and the grip value as a RobotCmd message.
    @staticmethod
    def calculate_control_signal(self):
    
        new_velocity = self.__k * (self.__desired_pos - self.__current_pos)
        new_velocity[0] += self.__vel_x_offset

        # set velocity to maximum when velocity is to high  
        new_velocity[new_velocity > self.__max_vel] = self.__max_vel[new_velocity > self.__max_vel]
        new_velocity[new_velocity < -self.__max_vel] = -self.__max_vel[new_velocity < -self.__max_vel]
       
        #check if the velocity jump is to high
        self.__velocity[new_velocity - self.__velocity > self.__max_acc] += self.__max_acc[new_velocity - self.__velocity > self.__max_acc]
        self.__velocity[self.__velocity - new_velocity > self.__max_acc] -= self.__max_acc[self.__velocity - new_velocity > self.__max_acc]
        self.__velocity[abs(new_velocity - self.__velocity) <= self.__max_acc] = new_velocity[abs(new_velocity - self.__velocity) <= self.__max_acc]   

        # create ros2 message
        robot_command = RobotCmd()
        robot_command.activate_gripper = self.__grip
        robot_command.vel_x, robot_command.vel_y, robot_command.vel_z = self.__velocity
        return robot_command
    
## Main function to create and run the Position Controller node.
def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionController()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()