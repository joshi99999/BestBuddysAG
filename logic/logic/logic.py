#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import PosVelClass, RobotPos, ConCmd
from time import time


class Controller(Node):
    def __init__(self):
        super().__init__('logic')

        self.box_cat_x = 0.04
        self.box_unicorn_x = 0.09
        self.box_y = 0.1
        self.wait = [0.06, 0.02, 0.06]
        self.pickup_z = 0.07
        self.y_enabled_z = 0.03

        self.subscriber = self.create_subscription(PosVelClass, 'pos_vel_class', self.object_callback, 10)
        self.subscriptionCurrent = self.create_subscription(RobotPos, 'robot_position', self.robotPosition_callback, 10)
        self.publisher = self.create_publisher(RobotPos, 'controller_command', 10)    

        self.error = False

        self.msg_out = ConCmd()
        self.msg_out.vel_x, self.msg_out.grip = 0.0, False

        self.time = 0.0
        self.type = None    #id: 0= Katze, 1 = Einhorn
        self.state = 'init'

        self.init_time = None
        self.init_duration = 5
        self.position = np.zeros(2, dtype=np.float64)

    def object_callback(self, msg):
        if 0 < self.msg_out.vel_x or self.error:
            return
        self.msg_out.vel_x = msg.vel_x
        self.time = msg.time
        self.type = msg.result
        self.position[:] = msg.pos_x, msg.pos_y
    
    def robotPosition_callback(self, msg_in):
        if self.error:
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
                    self.wait[2] += msg_in.pos_z
                    self.pickup_z += msg_in.pos_z
                    self.y_enabled_z += msg_in.pos_z
                    self.state = 'reference_z'
                
            case 'reference_z':
                if self.init_time is None:
                    self.msg_out.pos_z = msg_in.pos_z
                    self.init_time = time()
                
                if time() - self.init_time < self.init_duration:
                    self.msg_out.pos_x, self.msg_out.pos_y = msg_in.pos_x - 0.01, msg_in.pos_y - 0.01
                    self.publisher.publish(self.msg_out)
                else:
                    self.box_cat_x += msg_in.pos_x
                    self.box_unicorn_x += msg_in.pos_x
                    self.box_y += msg_in.pos_y
                    self.wait[0] = msg_in.pos_x
                    self.wait[1] = msg_in.pos_y
                    self.msg_out.pos_x, self.msg_out.pos_y, self.msg_out.pos_z = self.wait
                    self.publisher.publish(self.msg_out)
                    self.state = 'wait'

            case 'wait':
                if 0 < self.msg_out.vel_x:
                    self.msg_out.pos_y, self.msg_out.pos_z = self.position[1], self.wait[2]
                    self.state = 'fetch_x'

            case 'fetch_x':
                self.msg_out.pos_x = self.position[0] + (time() - self.time) * self.msg_out.vel_x
                if np.linalg.norm([msg_in.pos_x - self.msg_out.pos_x, msg_in.pos_y - self.msg_out.pos_y, msg_in.pos_z - self.msg_out.pos_z]) < 0.001:   
                    if self.msg_out.pos_z < self.pickup_z:
                        self.msg_out.pos_z, self.msg_out.pos_y = self.pickup_z, msg_in.pos_y
                    else:
                        self.msg_out.grip, self.msg_out.vel_x, self.msg_out.pos_x, self.msg_out.pos_z = True, 0, msg_in.pos_x, self.wait[2]
                        self.state = 'lifting'
                self.publisher.publish(self.msg_out)

            case 'lifting':
                if msg_in.pos_z - self.wait[2] < 0.001:
                    self.msg_out.pos_x = self.box_cat_x if self.type == 0 else self.box_unicorn_x
                    self.msg_out.pos_z = self.y_enabled_z
                    self.publisher.publish(self.msg_out)
                    self.state = 'y_enabling'

            case 'y_enabling':
                if msg_in.pos_z - self.y_enabled_z < 0.001:
                    self.msg_out.pos_y, self.msg_out.pos_z = self.box_y, msg_in.pos_z
                    self.publisher.publish(self.msg_out)
                    self.state = 'xy_moving'

            case 'xy_moving':
                if np.linalg.norm([msg_in.pos_x - self.msg_out.pos_x, msg_in.pos_y - self.msg_out.pos_y]) < 0.001:
                    self.msg_out.grip, self.msg_out.pos_x, self.msg_out.pos_y = False, msg_in.pos_x, msg_in.pos_y
                    self.publisher.publish(self.msg_out)
                    self.msg_out.pos_x, self.msg_out.pos_y = self.wait[0], self.wait[1]
                    self.publisher.publish(self.msg_out)
                    self.state = 'y_disabling'

            case 'y_disabling':
                if msg_in.pos_y - self.wait[1] < 0.001:
                    self.msg_out.pos_z = self.wait[2]
                    self.publisher.publish(self.msg_out)
                    self.state = 'wait'
                

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
