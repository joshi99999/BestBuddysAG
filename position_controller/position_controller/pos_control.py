#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd

class PositionController(Node):
    
    def __init__(self):
        super().__init__('control_roboter_position')

        #gain, could be even higher
        #self.kp = 3
        self.k = np.array([0.5, 1, 1])
        #max und min velocity
        self.min_vel = 0.01
        self.max_vel = 0.03

        self.accuracy = 0.002

        self.desired_pos = np.zeros(3, dtype=np.float32)
        self.current_pos = np.zeros(3, dtype=np.float32)
        self.difference_pos = np.zeros(3, dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)

        queue = 10
        self.subscriptionCurrent = self.create_subscription(RobotPos, 'robot_position', self.robotPostion_callback, queue)
        self.subscriptionDesired = self.create_subscription(RobotPos, 'robot_reference_position', self.desiredPosition_callback, queue)

        self.publisher = self.create_publisher(RobotCmd, 'robot_command', queue)         

    def desiredPosition_callback(self, msg):
        self.desired_pos[:] = msg.pos_x, msg.pos_y, msg.pos_z
    
    def robotPostion_callback(self, msg):
        self.current_pos[:] = msg.pos_x, msg.pos_y, msg.pos_z

        self.difference_pos[:] = self.desired_pos - self.current_pos


        #https://roboticsbackend.com/ros2-python-publisher-example/
        cmd = self.calculate_control_signal()
        self.publisher.publish(cmd)
        # self.position_publisher.publish(msg.RobotCmd(vel_x = positionError[0],vel_y =  positionError[1], vel_z =  positionError[2], activate_gripper = gripper))

        # Display the message on the console
        # TODO: nur publishen wenn vel != 0 ist!!!
        self.get_logger().info('Publishing: "%s"' % cmd)



    def calculate_control_signal(self):
        # velocity depends on difference
        self.velocity = self.k * self.difference_pos
        
        # set velocity to minimum when velocity is to low
        self.velocity[self.velocity.any() > 0 and self.velocity.any() < self.min_vel] = self.min_vel
        self.velocity[self.velocity.any() < 0 and self.velocity.any() > -1 * self.min_vel] = -1 * self.min_vel
        # set velocity to macimum when velocity is to high
        self.velocity[self.velocity > self.max_vel] = self.max_vel
        self.velocity[self.velocity < -1 * self.max_vel] = -1 * self.max_vel
        # set velocity to zero when accuracy is reached
        self.velocity[abs(self.difference_pos) < self.accuracy] = 0.0
        # create ros2 message
        robot_command = RobotCmd()
        robot_command.activate_gripper = False
        robot_command.vel_x, robot_command.vel_y, robot_command.vel_z = self.velocity
        return robot_command
    
def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionController()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()