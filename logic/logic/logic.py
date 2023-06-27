#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import PosVelClass, RobotPos, ConCmd
from time import time

POS_BOX_CAT = []
POS_BOX_UNICORN = []
POS_ERROR = [0, 0, 0]
POS_WAIT = []

class Controller(Node):
    def __init__(self):
        super().__init__('logic')

        self.subscriber = self.create_subscription(PosVelClass, 'pos_vel_class', self.object_callback, 10)
        self.subscriptionCurrent = self.create_subscription(RobotPos, 'robot_position', self.robotPostion_callback, 10)
        self.publisher = self.create_publisher(RobotPos, 'robot_reference_position', 10)    

        self.error = False
        self.msg_out = ConCmd()

        self.time = 0.0
        self.type = None    #id: 0= Katze, 1 = Einhorn
        self.state = 'init'
        self.target = np.zeros(3, np.float64)

        self.init_time = None
        self.init_duration = 5
        self.reference_position = np.zeros(3, dtype=np.float64)
        self.position = np.zeros(2, dtype=np.float64)

    def object_callback(self, msg):
        if 0 < self.velocity or self.error:
            return
        self.locked = True
        self.position[:] = msg.pos_x, msg.pos_y
        self.velocity = msg.vel_x
        self.time = msg.time
        self.type = msg.result
    
    def robotPosition_callback(self, msg_in):
        if self.error:
            return        
        
        self.msg_out.vel_x, self.msg_out.grip = .0, False

        position = np.array([self.msg_out.pos_x, self.msg_out.pos_y, self.msg_out.pos_z], dtype=np.float64)
        if 0.001 < np.linalg.norm(self.target - position):
            return

        match self.state:
            case 'init':
                if self.init_time is None:
                    self.msg_out.pos_x, self.msg_out.pos_y = msg_in.pos_x, msg_in.pos_y
                    self.init_time = time()
                
                if time() - self.init_time < self.init_duration:
                    self.msg_out.pos_z = msg_in.pos_z - 0.01
                    self.publisher.publish(self.msg_out)
                else:
                    self.init_time = None
                    self.reference_position[3] = msg_in.pos_z
                    self.state = 'reference_z'
                
            case 'reference_z':
                if self.init_time is None:
                    self.msg_out.pos_z = msg_in.pos_z
                    self.init_time = time()
                
                if time() - self.init_time < self.init_duration:
                    self.msg_out.pos_x, self.msg_out.pos_y = msg_in.pos_x - 0.01, msg_in.pos_y - 0.01
                    self.publisher.publish(self.msg_out)
                else:
                    self.reference_position[0:2] = msg_in.pos_x, msg_in.pos_y
                    self.state = 'reference'

            case 'reference':
                self.msg_out.pos_x, self.msg_out.pos_y, self.msg_out.pos_z = POS_WAIT
                self.publisher.publish(self.msg_out)
                self.state = 'wait'

            case 'wait':
                if 0 < self.msg_out.vel_x:
                    self.state = 'fetch'
            
            case 'fetch_yz':
                x = self.position[0] + (time() - self.time) * self.msg_out.vel_x
                if 0.001 < np.linalg.norm(x - [self.msg_out.pos_x, self.msg_out.pos_y, self.msg_out.pos_z]):
                    self.msg_out.pos_x, self.msg_out.pos_y = x
                    self.msg_out.pos_z = POS_WAIT[2]
                    self.publisher.publish(self.msg_out)
                    self.msg_in.pos_y = 
                else:

            case 'fetch_x':

            case 'object_yz':
                
            case 'object':
            case 'lifted':
            case 'y_enabled':
            case 'box':
            case 'wait_xy':

                

                self.target[:] = POS_WAIT

                
                
    def pickupObject(self, position):
        if 0.001 < abs(position.pos_x - self.velocity * time()) and 0.001 < abs(self.pos_y - position.pos_y):


        if position[2] < 0.03:
            msg = RobotPos()
            msg.pos_x, msg.pos_y, msg.pos_z = self.velocity * time() + self.pos_x, self.pos_y, position[2] 
            self.publisher.publish(msg)
        


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
