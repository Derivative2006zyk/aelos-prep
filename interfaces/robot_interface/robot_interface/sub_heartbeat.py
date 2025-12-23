import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HeartbeatSubscriber(Node):
    def __init__(self):
        super().__init__('heartbeat_subscriber_b')
        #subscriber's init,mean:/heartbeat,type:String,laugh:10
        self.subscription = self.create_subscription(String,'heartbeat',self.lisener_callback,10)
        self.subscription  #prevent aciidental garbage collection
    def lisener_callback(self,msg):
        self.get_logger().info(f'The student B accept the heartbeat:{msg.data}')
def main(args=None):
    rclpy.init(args=args)
    subscriber = HeartbeatSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()