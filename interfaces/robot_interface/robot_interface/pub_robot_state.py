import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        #publisher's init,mean:/robot_state,type:String,laugh:10
        self.publisher_ = self.create_publisher(String,'/robot_state_publisher',10)
        self.timer = self.create_timer(3.0,self.timer_callback)
        self.states = ['IDLE','RUNNING','ERROR','FALLEN'] #the list of robot_state
        self.state_index = 0
    def timer_callback(self):
        msg = String()
        msg.data = self.states[self.state_index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'publish robot_state:{msg.data}')
        self.state_index = (self.state_index + 1) % 4 #cycle switching state
def main(args=None):
    rclpy.init(args=args)
    publisher = RobotStatePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()