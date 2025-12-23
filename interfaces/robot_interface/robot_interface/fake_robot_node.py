# V1:Compatible FSMv1 Controller
# ✅ Keep the source code of V0 and only comment it out
# ✅ New subscription /robot_state from FSM'status
# ✅ Execute tasks based on changes in status
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random  # Keep to random status
class FakeRobotNode(Node):
    def __init__(self):
        super().__init__('fake_robot_node')
        # /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
         )
        self.get_logger().info("fake_robot_node Started,listening /cmd_vel...")
        #  /robot_state 
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        # 🔴 
        # self.create_timer(2.0, self.publish_random_state)
        # self.get_logger().info("robot_state")
        # ✅ NEW:Sub FSM publishing /robot_state
        self.robot_state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            10
        )
        self.current_state = "UNKNOWN"
        self.get_logger().info("🟢 CONTROL by FSM /robot_state,this node only responds to changes")
    def cmd_vel_callback(self, msg: Twist):
        """
        Analyze and print motion instructions
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        if linear_x > 0:
            action = f"Forward (LINEAR_velocity={linear_x:.2f} m/s)"
        elif linear_x < 0:
            action = f"Backward (LINEAR_velocity={abs(linear_x):.2f} m/s)"
        elif angular_z > 0:
            action = f"Left_turn (ANGULAR_velocity={angular_z:.2f} rad/s)"
        elif angular_z < 0:
            action = f"Right_turn (ANGULAR_velocity={abs(angular_z):.2f} rad/s)"
        else:
            action = "Stop"
        # Determine whether to take action based on the current status
        if self.current_state in ['FACE_TASK', 'AVOID_TASK', 'RUNNING', 'CALIB']:
            self.get_logger().info(f"[{self.current_state}] Perform{action}")
        else:
            self.get_logger().warn(f"[{self.current_state}] The current state ignores movement instructions{action}")
    def robot_state_callback(self, msg: String):
        """
        recieve FSM publish-status
        """
        new_state = msg.data
        old_state = self.current_state
        if new_state != old_state:
            self.current_state = new_state
            timestamp = self.get_clock().now().to_msg().sec
            self.get_logger().info(f"🔄 UPDATE state[{old_state}] → [{new_state}] (T={timestamp})")
            # Imitate initialization behavior of each state
            if new_state == 'CALIB':
                self.simulate_calibration()
            elif new_state == 'FACE_TASK':
                self.simulate_face_detection()
            elif new_state == 'AVOID_TASK':
                self.enable_obstacle_avoidance()
            elif new_state == 'END':
                self.shutdown_actions()
    def simulate_calibration(self):
        self.get_logger().info("⚙️  Virtual Calibration beginning,IMU zeroing,Servo Initialization...FINISHED")
    def simulate_face_detection(self):
        self.get_logger().info("📷 THE FACE Detection module is activated,BEGINS scanning the area ahead...")
    def enable_obstacle_avoidance(self):
        self.get_logger().info("🛡️  Start Automatic obstacle avoidance simulation:radar data stream ready,path planing enabled")
    def shutdown_actions(self):
        self.get_logger().info("🛑 All tasks completed,execute shutdown process:Motor Shutdown,Sensor Shutdown")
    # V0
'''
    def publish_random_state(self):
        states = ["IDLE", "RUNNING", "ERROR", "FALLEN"]
        random_state = random.choice(states)
            
        msg = String()
        msg.data = random_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"📢 [imitate] Self publishing status: {random_state}")
'''
    
def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()

    def publish_robot_state(self):
        """Regularly publish robot_status"""
        state_msg = String()

        # Random switch status
        state_msg.data = random.choice(self.state_list)
        self.robot_state_pub.publish(state_msg)
        self.get_logger().info(f"Publish status{state_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotNode()
    rclpy.spin(node)  # Keep Node On
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()