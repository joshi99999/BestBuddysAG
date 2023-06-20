#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd


class PositionController(Node):
    
    def __init__(self):
        super().__init__('control_roboter_position')
        
        self.k = np.array([3, 3, 3])

        self.max_vel = np.array([0.03, 0.04, 0.05], dtype=np.float64)
        self.max_acc = np.array([0.005, 0.005, 0.02], dtype=np.float64)

        self.desired_pos = np.zeros(3, dtype=np.float64)
        self.current_pos = np.zeros(3, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)

        self.subscriptionCurrent = self.create_subscription(RobotPos, 'robot_position', self.robotPostion_callback, 10)
        self.subscriptionDesired = self.create_subscription(RobotPos, 'robot_reference_position', self.desiredPosition_callback, 10)

        self.publisher = self.create_publisher(RobotCmd, 'robot_command', 10)


    def desiredPosition_callback(self, msg):
        self.desired_pos[:] = msg.pos_x, msg.pos_y, msg.pos_z

    
    def robotPostion_callback(self, msg):
        self.current_pos[:] = msg.pos_x, msg.pos_y, msg.pos_z

        cmd = self.calculate_control_signal()
        self.publisher.publish(cmd)

        self.get_logger().info('Publishing velocities: ' + str(cmd))


    def calculate_control_signal(self):

        # velocity depends on difference
        new_velocity = self.k * (self.desired_pos - self.current_pos)

        # set velocity to maximum when velocity is to high  
        new_velocity[new_velocity > self.max_vel] = self.max_vel[new_velocity > self.max_vel]
        new_velocity[new_velocity < -self.max_vel] = -self.max_vel[new_velocity < -self.max_vel]
       
        #check if the velocity jump is to high
        self.velocity[new_velocity - self.velocity > self.max_acc] += self.max_acc[new_velocity - self.velocity > self.max_acc]
        self.velocity[self.velocity - new_velocity > self.max_acc] -= self.max_acc[self.velocity - new_velocity > self.max_acc]
        self.velocity[abs(new_velocity - self.velocity) <= self.max_acc] = new_velocity[abs(new_velocity - self.velocity) <= self.max_acc]   

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