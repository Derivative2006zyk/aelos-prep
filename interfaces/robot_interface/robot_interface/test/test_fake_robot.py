import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pytest
from robot_interface.fake_robot_node import FakeRobotNode

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.robot_state_sub = self.create_subscription(
            String, "/robot_state", self.state_callback, 10
        )
        self.received_state = None
    def state_callback(self, msg):
        self.received_state = msg.data
    def publish_cmd_vel(self, linear_x, angular_z):
        """pub cmd_vel information"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)
# initialize ROS2 source (before all of tests)
@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
# test 1: recieve STOP command(linear + augular = 0), node print "stop" log
def test_stop_command(ros_context, capsys):

    # ON Fake Robot_node and teat_node
    fake_node = FakeRobotNode()
    test_node = TestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(fake_node)
    executor.add_node(test_node)
    # publish STOP command
    test_node.publish_cmd_vel(linear_x=0.0, angular_z=0.0)
    # trigger callback(simulate ROS2 roop)
    executor.spin_once(timeout_sec=1.0)
    # check node_print
    captured = capsys.readouterr()
    assert "recieve stop command" in captured.out
    # break the node
    fake_node.destroy_node()
    test_node.destroy_node()
# test 2: recieve FORWARD command, node print "forward" log

def test_forward_command(ros_context, capsys):
    fake_node = FakeRobotNode()
    test_node = TestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(fake_node)
    executor.add_node(test_node)

    test_node.publish_cmd_vel(linear_x=0.5, angular_z=0.0)
    executor.spin_once(timeout_sec=1.0)

    captured = capsys.readouterr()
    assert "recieve forward_command,linear =0.5m/s" in captured.out

    fake_node.destroy_node()
    test_node.destroy_node()

# test 3: recieve LEFT command, node print "left" log 

def test_turn_left_command(ros_context, capsys):
    fake_node = FakeRobotNode()
    test_node = TestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(fake_node)
    executor.add_node(test_node)

    test_node.publish_cmd_vel(linear_x=0.0, angular_z=0.3)
    executor.spin_once(timeout_sec=1.0)

    captured = capsys.readouterr()
    assert "recieve left_command,augular=0.3rad/s" in captured.out

    fake_node.destroy_node()
    test_node.destroy_node()

# test 4:publish status in regular time,and check if the status is on the list

def test_state_publish(ros_context):
    fake_node = FakeRobotNode()
    test_node = TestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(fake_node)
    executor.add_node(test_node)
    # wait the timer turn on(make 5second and 1second to buffer)
    executor.spin_once(timeout_sec=6.0)
    # check if the status that was recieved is on the allowed list

    allowed_states = ["IDLE", "RUNNING", "ERROR", "FALLEN"]
    assert test_node.received_state in allowed_states

    fake_node.destroy_node()
    test_node.destroy_node()
