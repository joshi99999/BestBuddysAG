import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd

class PositionController(Node):
    
    def __init__(self):
        super().__init__('Control Roboter Position')

        #gain, could be even higher
        self.kp = 3
        #boarders
        self.max = 50
        self.min = 1

        #later the Position[X,Y,Z]
        self.postionValue = [0,0,0]

        self.gripper = False

        #https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
        #create a subscriber to subscribe the roboter position for the thtee axis
        #float32 pos_x, float32 pos_y, float32 pos_z
        self.subscription = self.create_subscription(RobotPos, 'RobotPos', self.robotPostion_callback, 10)
        
        #self.subscription = self.create_subsciption(, , ,10)

        #create a publisher to publish the positionError for the three axis
        #float32 vel_x, float32 vel_y, float32 vel_z, bool activate_gripper
        self.publisher = self.create_publisher(RobotCmd, 'RobotCmd', 10) 
        
        self.current_pos = RobotPos()
        self.desired_pos = RobotPos()

       

    #https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Simple-Publisher-Subscriber.html    
    def robotPostion_callback(self, msg):
        #positionError has 3 possible positionErrors: X,Y,Z
        # msg.data should be the latest position from the axis
        positionError = [self.postionValue[i] - msg.data[i] for i in range(2)]

        #if Error is to small, it should not coreccte anything, otherwise it could begin to swing
        for error in positionError:
            if error < min:
                error = 0
            #max border if Error is to big, otherwise it could overshoot    
            elif positionError > max:
                error = self.max

        positionError = [self.kp * positionError[0], self.kp * positionError[1], self.kp * positionError[2]]
        
        #if msg.data[2] < 200:
         #  self.gripper = True
        #elif msg.data[0] == 500 & msg.data[1] == 500:
         #   self.gripper = False
        #elif msg.data[0] == 400 & msg.data[1] == 500:
         #   self.gripper = False
        
        #https://roboticsbackend.com/ros2-python-publisher-example/
        self.publisher.publish(msg.RobotCmd(vel_x = positionError[0],vel_y =  positionError[1], vel_z =  positionError[2], activate_gripper = gripper))
        # self.position_publisher.publish(msg.RobotCmd(vel_x = positionError[0],vel_y =  positionError[1], vel_z =  positionError[2], activate_gripper = gripper))

        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % positionError)

    def main():
        #https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
        rclpy.init(args=args)
        positionControl = PositionController(100, 1, 10)
        rclpy.spin(positionControl)
        positionControl.destroy_node()
        rclpy.shutdown()

    if __name__ == "__main__":
     main()

