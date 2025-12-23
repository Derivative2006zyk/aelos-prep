import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        #publisher's init,mean:/cmd_vel,type:Twist,laugh:10
        self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(2.0,self.timer_callback)
        self.twist = Twist()
    def timer_callback(self):
        self.twist.linear.x = 0.2 #linear velocity
        self.twist.angular.z = 0.5 #angular velocity
        self.publisher_.publish(self.twist)
        self.get_logger().info(f'called cmd_vel:linear.x={self.twist.linear.x},angular.z={self.twist.angular.z}')
def main(args=None):
    rclpy.init(args=args)
    publisher = CmdVelPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()