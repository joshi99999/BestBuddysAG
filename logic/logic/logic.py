#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd

class Controller(Node):
    def __init__(self):
        super().__init__('logic')

        self.desired_pos = np.zeros(3, dtype=np.float32)
        self.min_movement_x = 0.0
        self.max_movement_x = 10.0

        self.subscriptionDesired = self.create_subscription(RobotPos, 'robot_reference_position', self.desiredPosition_callback, 10)
        self.publisher = self.create_publisher(RobotCmd, 'robot_command', 10)    
        #var_pos_belt is only True if the robot is at a trayer
        self.var_pos_trayer = False
        #var_pos_trayer is only True if the robot is at the belt
        self.var_pos_belt = False     


        #id: 0= Katze, 1 = Einhorn

    def desiredPosition_callback(self, msg):
        pos = self.desicion(msg.id)
        self.publisher.publish(pos)
        self.get_logger().info('Publishing: "%s"' % pos)

    def desicion(self, id):
        if (id == 0 & self.var_pos_trayer == True): 
            self.catTrayer()
        if (id == 1 & self.var_pos_trayer == True):
            self.unicornTrayer()
             
    def unicornTrayer(self):
        #TODO x und y positionen bestimmen und anpassen 
        self.var_pos_trayer = False
        self.var_pos_belt = True
        xPosUnicorn = 0.5
        yPosUnicorn = 0.5
        zPosUnicorn = 0.06
        activate_gripper = True
        self.desired_pos = xPosUnicorn, yPosUnicorn, zPosUnicorn
        self.desiredPosition_callback(self.desired_pos)

    def catTrayer(self):
        #TODO x und y positionen bestimmen und anpassen 
        self.var_pos_trayer = False
        self.var_pos_belt = True
        xPosCat = 0.5
        yPosCat = 0.5
        zPosCat = 0.06
        activate_gripper = True
        self.desired_pos = xPosCat, yPosCat, zPosCat
        self.desiredPosition_callback(self.desired_pos)
    
    def waitPosition(self):
        #TODO x und y positionen bestimmen und anpassen 
        xPosWait = 0.5
        yPosWait = 0.5
        zPosWait = 0.06
        self.desired_pos[:] = xPosWait, yPosWait, zPosWait

    def errorPosition(self):
        #TODO x, y und z positionen bestimmen und anpassen
        xPosError = 0.5
        yPosError = 0.5
        zPosError = 0.03
        self.desired_pos[:] = xPosError, yPosError, zPosError

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
