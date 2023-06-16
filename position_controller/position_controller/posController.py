#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd

class PositionController(Node):
    
    def __init__(self):
        super().__init__('posController')

        #gain, could be even higher
        #self.kp = 3
        self.gripper = False

        #https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
        #create a subscriber to subscribe the roboter position for the thtee axis
        #float32 pos_x, float32 pos_y, float32 pos_z
        self.currentPositionSub = self.create_subscription(RobotPos, 'RobotPos', self.robotPostion_callback, 10)
        
        self.subscriptionDesired = self.create_subscription(RobotPos, 'robot_reference_position', self.robotPostion_callback, 10)

        #create a publisher to publish the positionError for the three axis
        #float32 vel_x, float32 vel_y, float32 vel_z, bool activate_gripper
        self.publisher = self.create_publisher(RobotCmd, 'RobotCmd', 10) 

        self.currentPosition = RobotPos
        #noch ändern
        self.desiredPosition = RobotPos

    def posDifference(self):

        robot_cmd = RobotCmd
        kp = 3

        xDifference = self.desiredPosition.get_x - self.currentPosition.get_z
        if xDifference < 0.001 or xDifference > -0.001:
            xDifference = 0.0

        yDifference = self.desiredPosition.get_y - self.currentPosition.get_z
        if yDifference < 0.001 or yDifference > -0.001:
            yDifference = 0.0

        zDifference = self.desiredPosition.get_z - self.currentPosition.get_z
        if zDifference < 0.001 or zDifference > -0.001:
            zDifference = 0.0

        robot_cmd.vel_x = xDifference * kp
        robot_cmd.vel_y = yDifference * kp
        robot_cmd.vel_z = zDifference * kp
        robot_cmd.activate_gripper = False
        self.positionError = robot_cmd

        return robot_cmd



    #https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Simple-Publisher-Subscriber.html    
    def robotPostion_callback(self):

        newPos = self.posDifference()

        #positionError = [self.kp * self.xDifference, self.kp * self.yDifference, self.kp * self.zDifference]
        self.position_publisher_.publish(RobotCmd(vel_x = self.positionError[0],vel_y =  self.positionError[1], vel_z =  self.positionError[2], activate_gripper = self.gripper))

        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.positionError) 


def main():

    #https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
    rclpy.init(args=None)
    posController = PositionController()
    rclpy.spin(posController)
    posController.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()