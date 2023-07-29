#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd, ConCmd
from time import time

## \class PositionController
# A position controller for the portal robot, to calculate a steady new velocity for the axis of the robot in dependence of the current position and the desired position.class PositionController(Node):
class PositionController(Node):
    
    def __init__(self):
        ## Initializes a new Position Controller instance.
    #  @param k Proportional gain value for the velocity calculation.
    #  @param max_vel Array with three values for the maximum velocity of each step motor.
    #  @param max_acc Array with thee values for the maximum velocity of each step motor.
    #  @param desired_pos Desired position for each step motor.
    #  @param current_pos Current position for each step motor.
    #  @param velocity Current velocity for each step motor.
    #  @param vel_x_offset Offset for the x velocity.
    #  @param grip Boolean value to control the robot gripper.
    # Subscribs the ROS2 message of the current position of the robot from the RobotPos and the desired position from the ConCmd topic.
    # Publishes the ROS2 message of the calculated velocity to the RobotCmd topic.
    
        super().__init__('control_roboter_position')
        
        self.k = np.array([3, 3, 3], dtype=np.float64)

        self.max_vel = np.array([0.03, 0.04, 0.05], dtype=np.float64)
        self.max_acc = np.array([0.005, 0.005, 0.02], dtype=np.float64)

        self.desired_pos = np.zeros(3, dtype=np.float64)
        self.current_pos = np.zeros(3, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        
        self.vel_x_offset = 0.0
        self.grip = False

        self.subscriptionCurrent = self.create_subscription(RobotPos, 'robot_position', self.robotPostion_callback, 10)
        self.subscriptionDesired = self.create_subscription(ConCmd, 'controller_command', self.controllerCommand_callback, 10)

        self.publisher = self.create_publisher(RobotCmd, 'robot_command', 10)

    ## Callback function to get the desired position X, Y and Z and the offset for the x velocity.
    #  @param msg The ConCmd message containing the desired position and velocity offset.
    @staticmethod
    def controllerCommand_callback(self, msg):
        self.desired_pos[:] = [msg.pos_x, msg.pos_y, msg.pos_z]
        self.vel_x_offset, self.grip = msg.vel_x, msg.grip
    
    ## Callback function to get the current position X, Y and Z and publish the velocity in the consol.
    #  @return robot_command Returns the calculated velocity for each step motor and the grip value as a RobotCmd message.
    @staticmethod
    def robotPostion_callback(self, msg):
        self.current_pos[:] = msg.pos_x, msg.pos_y, msg.pos_z

        cmd = self.calculate_control_signal()
        self.publisher.publish(cmd)

        self.get_logger().info('Publishing velocities: ' + str(cmd))

    ## Calculates the new velocity for each step motor in dependence of the current position and the desired position.
    # @param new_velocity New velocity for each step motor which is calculated by the position difference times the gain value.
    # For the velocity in x direction the offset has to be added because with this the system is in rest position.
    # The velocity is set to the maximum velocity when the velocity is higher than the maximum velocity.
    # The velocity is set to the maximum acceleration when the velocity jump is higher than the maximum acceleration.
    # @return robot_command Returns the calculated velocity for each step motor and the grip value as a RobotCmd message.
    @staticmethod
    def calculate_control_signal(self):
    
        new_velocity = self.k * (self.desired_pos - self.current_pos)
        new_velocity[0] += self.vel_x_offset

        # set velocity to maximum when velocity is to high  
        new_velocity[new_velocity > self.max_vel] = self.max_vel[new_velocity > self.max_vel]
        new_velocity[new_velocity < -self.max_vel] = -self.max_vel[new_velocity < -self.max_vel]
       
        #check if the velocity jump is to high
        self.velocity[new_velocity - self.velocity > self.max_acc] += self.max_acc[new_velocity - self.velocity > self.max_acc]
        self.velocity[self.velocity - new_velocity > self.max_acc] -= self.max_acc[self.velocity - new_velocity > self.max_acc]
        self.velocity[abs(new_velocity - self.velocity) <= self.max_acc] = new_velocity[abs(new_velocity - self.velocity) <= self.max_acc]   

        # create ros2 message
        robot_command = RobotCmd()
        robot_command.activate_gripper = self.grip
        robot_command.vel_x, robot_command.vel_y, robot_command.vel_z = self.velocity
        return robot_command

## \file
# \brief Main function to create and run the Position Controller node.
#  
def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionController()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()