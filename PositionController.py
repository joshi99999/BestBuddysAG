import rclpy
import rclpy.node as Node
import ro45_portalrobot_interface.msg as msg

class PositionController(Node):
    
    def __init__(self):
        super().__init__('Control Roboter Position')

        #gain
        self.kp = 10
        
        self.max = 100
        self.min = 1

        #later the Position[X,Y,Z]
        self.postionValue = [0,0,0]

        #https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
        #create a subscriber to subscribe the roboter position for the thtee axis
        #float32 pos_x, float32 pos_y, float32 pos_z
        self.subscription = self.create_subscription(msg.RobotPos, 'RobotPos', self.robotPostion_callback, 10)

        #create a publisher to publish the positionError for the three axis
        #float32 vel_x, float32 vel_y, float32 vel_z, bool activate_gripper
        self.publisher = self.create_publisher(msg.RobotCmd, 'RobotCmd', 10) 

       

    #https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Simple-Publisher-Subscriber.html    
    def robotPostion_callback(self, msg):
        #positionError has 3 possible positionErrors: X,Y,Z
        positionError = [self.postionValue[i] - msg.data[i] for i in range(2)]

        #if Error is to small, it should not coreccte anything, otherwise it could begin to swing
        if positionError < min:
            positionError = 0
        #max border if Error is to big, otherwise it could overshoot    
        elif positionError > max:
            positionError = self.max

        positionError = [self.kp * positionError[0], self.kp * positionError[1], self.kp * positionError[2]]

        gripper = False
        #https://roboticsbackend.com/ros2-python-publisher-example/
        self.position_publisher_.publish(msg.RobotCmd(vel_x = positionError[0],vel_y =  positionError[1], vel_z =  positionError[2], activate_gripper = gripper))

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
