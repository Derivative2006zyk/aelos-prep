import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        #called/heartbeat,String,10
        self.publisher_= self.create_publisher(String,'/heartbeat',10)
        self.timer = self.create_timer(1.0,self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Heartbeat:{self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'call:{msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = HeartbeatPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()